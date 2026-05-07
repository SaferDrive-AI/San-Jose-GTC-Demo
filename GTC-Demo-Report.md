# GTC-Demo-Report — San Jose GTC Demo 项目深度技术分析

**目标项目路径**：`/Users/happyfewmac/Desktop/Inchor/San-Jose-GTC-Demo`
**最新分析日期**：2026-05-06
**分支**：`San-Jose-Demo-With-Linker-Terasim`
**分析框架**：Senior Research Analyst — Deconstruction → Sourcing → Synthesis → Risk/Gap

---

## TL;DR / Executive Summary

- **项目本质**：基于 SUMO + TraCI 的"障碍物感知信号灯优化"交通仿真平台。以 San Jose 市中心 **S Market St & W Santa Clara St** 路口为研究对象，量化抛锚车辆（stalled vehicle）造成的延误，并验证一套自适应信号配时方案能在多大程度上缓解延误。
- **本分支主要新增**：从 LinkVision 摄像头检测的**真实抛锚事件 JSON**直接驱动仿真——一个新的 Python 包 `linkvision_terasim/` + 两个 CLI 入口（`run_linkvision_replay.py` 走 main.py 子进程；`run_linkvision_terasim.py` 走 TeraSim 框架）+ 5 套单元测试 + 一个按方向拆分 tripinfo 的分析脚本。
- **核心定量结果**（合成场景）：默认 TLS + WB 直行抛锚使平均延误从 26.1s 升到 31.8s（+22%）；切到 Plan 4（EW 绿 28.2s）拉回到 28.3s（−11%）。
- **核心定量结果**（真实 LinkVision 事件 id=1521072，WB 抛锚车）：bench `org` 30.79s → dynamic 自动切 `-416901230#1_3`（exact_lane_program）27.74s，**−9.9% 延误，−11% 总等待时间**。
- **NVIDIA GTC 演示价值**：完整闭环 *相机 → 单应矩阵 → SUMO/TeraSim 数字孪生 → 信号优化 → 评估 / 4K 60fps 视频*，适合在 GTC 演示"AI 视觉 + 物理仿真 + 自适应控制"的端到端故事。

---

## 1. 为什么需要这个项目（Why）

### 1.1 现实痛点
城市路口通行能力高度依赖每条车道是否畅通。一旦发生**单车事件（stalled / disabled vehicle）**，即抛锚、事故或临时停车，**该车道相当于瞬时被"截流"**，造成：

1. 该方向通行能力下降 → 排队累积；
2. 信号配时仍按"无事件"假设运行，**绿灯被浪费在不再使用的车道上**；
3. 二次拥堵向上游溢出，影响相邻路口（俗称 spillback）。

传统 SCATS / SCOOT 类自适应控制只对**整体流量**做反馈，难以在秒级感知"某条车道被堵"。本项目要回答的核心问题是：**当摄像头能在几秒内识别"WB 直行车道有抛锚车"时，是否可以即时切换到一个"对 EW 更友好"的信号方案，把损失补回来？**

### 1.2 演示动机（NVIDIA GTC）
项目目录名包含 "San-Jose-GTC-Demo"。本分支额外引入了 [TeraSim](https://github.com/SaferDrive-AI/TeraSim)（位于 `/Users/happyfewmac/Desktop/Inchor/Terasim`）作为可选的运行框架，并新增了与 LinkVision 摄像头数据的端到端绑定，目的是在 GTC 大会现场展示"**Sensor → AI 检测 → 数字孪生 → 控制决策 → 视觉化反馈**"的全链路能力。

### 1.3 学术 / 工程贡献
| 维度 | 说明 |
|---|---|
| **场景真实性** | 网络由 OpenStreetMap 抽出，叠加真实信号灯 phase 程序 |
| **量化基线** | 12 个抛锚位置（4 方向 × 3 车道：left/through/right）做静态 vs 动态对比，共 25 次仿真；新增"真实事件"维度（LinkVision 事件 1521072）|
| **可视化** | 通过 Real-ESRGAN 把背景瓦片 4× 超分（256→1024），辅以相机推进 / 旋转，实现 4K 演示级别 |
| **可测试性** | 把"切灯决策"从主循环抽出来做成纯函数 `choose_signal_plan()`，配 5 套 unittest |

---

## 2. 项目目的（What）

> 一句话：**用 SUMO 数字孪生量化"抛锚车 + 自适应配时"对路口性能的影响，且支持把 LinkVision 真实摄像头事件直接灌进来 replay，并产出可演示的 4K 视频与对比图表。**

具体可拆为五个目标：

1. **建模**：把 OSM 路网、信号灯多个 program、车流模型（vType + flow）、初始车队（initial_vehicles）、背景车 4000 辆全部装入 SUMO；
2. **激励**：通过 TraCI 在指定经纬度位置注入"零速静止车辆"作为障碍物——既支持手敲 GPS（合成场景），也支持从 LinkVision JSON 自动解析（真实事件）；
3. **控制**：依据 `--mode` 参数选择 `bench` / `opt` / `dynamic` 三种信号策略，dynamic 模式按障碍物所在 lane 自动切换 program；
4. **评估 + 可视化**：输出 JSON（含 `applied_tls_program` 实际切到了哪个程序）+ tripinfo XML + matplotlib 对比图 + 60fps 屏幕录制 → ffmpeg 编码为 H.264 MP4；
5. **复用 / 测试**：把决策逻辑抽成可单测的纯函数，并提供两个执行路径（直接 main.py 子进程 / TeraSim 框架托管）。

---

## 3. 怎么做（How）— 系统架构

```
                ┌──────────────────────────┐
                │  LinkVision Camera API    │  抛锚车检测
                │  (Inter_Cam_2_Stalled_Car) │  → response_stalled_car_detected.json
                └─────────────┬─────────────┘
                              │  + response_its_task_metadata.json (homography, resolution)
                              ▼
              ┌──────────────────────────────────────────┐
              │ linkvision_terasim/events.py              │
              │   bbox(0~1) → pixel (bottom-center)        │
              │   → homography 3×3 → (lat, lon)            │
              │   → RealWorldStalledVehicleEvent dataclass │
              └─────────────┬────────────────────────────┘
                            │
                ┌───────────┴────────────┐
                ▼                        ▼
   ┌────────────────────────┐  ┌──────────────────────────────┐
   │  run_linkvision_replay │  │  run_linkvision_terasim       │
   │  (subprocess → main.py)│  │  (in-proc TeraSim EnvTemplate)│
   └────────────┬───────────┘  └──────────────┬───────────────┘
                ▼                              ▼
   ┌────────────────────────┐  ┌──────────────────────────────┐
   │ main.py                │  │ terasim_runner.py             │
   │ SUMODelayCalculator    │  │  LinkVisionReplayEnv          │
   │  - convertGeo + add    │  │   on_start: add stalled veh   │
   │  - update_tls_program  │  │   on_step:  pin to (lane,pos) │
   │  - assist_stuck        │  │   uses choose_signal_plan()   │
   │  - rerouteTraveltime   │  │  prepare_sumolib_compatible_  │
   │  - applied_tls_program │  │   net_file (round float dur)  │
   └────────────┬───────────┘  └──────────────┬───────────────┘
                │                              │
                ▼                              ▼
   ┌────────────────────────┐  ┌──────────────────────────────┐
   │ SUMO 1.22+ / TraCI     │  │ TeraSim Simulator             │
   │ + osm.net.xml          │  │  outputs: tripinfo, fcd_all,  │
   │ + osm.tls.xml (multi-  │  │           run.log             │
   │   program)             │  │  output_path = outputs/       │
   │ + directional_traffic  │  │    linkvision_terasim/        │
   └────────────┬───────────┘  └──────────────┬───────────────┘
                │                              │
                ▼                              ▼
   ┌────────────────────────────────────────────────────────────┐
   │ traffic_data_analysis/                                     │
   │   delay_*.json + tls_program={tls_id, applied_program_id,  │
   │                              reason}                        │
   │   tripinfo/tripinfo_<eventID>_<mode>.xml                   │
   │   analyze_tripinfo_by_direction.py → bench vs dynamic 拆向 │
   │   plot_*.py → matplotlib 对比图                             │
   │   screenshots/ → ffmpeg → mp4 (60fps)                      │
   └────────────────────────────────────────────────────────────┘
```

子系统：

- **数字孪生网络**：两套 SUMO 网络
  - `san_jose_downtown_gtc/` — 单路口，定量评估
  - `san_jose_full_new/` — 完整 San Jose 大网络（≈3500 背景车 + 4K 瓦片底图），用于演示
- **网络合并工具**：`merge_networks.py` 用 *add-only* 策略把 downtown 网络的 edges/junctions 安全合并到 full 网络，避免版本冲突
- **背景瓦片增强**：`upscale_tiles.py` 用 Real-ESRGAN x4plus 把 256×256 JPEG 瓦片升到 1024×1024，让镜头拉到 zoom=10000 时仍清晰
- **真实事件接入层**：`linkvision_terasim/`（本分支新增，详见 §6.7）

---

## 4. 涉及到的关键技术栈

| 类别 | 技术 / 库 | 用途 |
|---|---|---|
| 交通仿真核心 | **SUMO 1.22+** | 微观交通仿真器；net、route、TLS、vehicle 物理 |
| 仿真控制 | **TraCI** | Python ↔ SUMO 的 TCP 接口 |
| **多智能体框架（可选）** | **TeraSim** (SaferDrive-AI) | `EnvTemplate / Simulator / VehicleFactory` 模板化封装 SUMO；可替代 main.py 自写主循环 |
| 路网数据 | OpenStreetMap → osm.net.xml | netconvert 把 OSM 转成 SUMO 网络 |
| 配时控制 | Actuated TLS + 多 program 切换 | 18-phase 100s 周期，per-lane program ID |
| 跟驰 / 换道模型 | Krauss + SL2015 (sublane) | 微观跟驰；亚车道横向行为 |
| **视觉投影** | **3×3 单应矩阵** | LinkVision 像素 (px,py) → GPS (lat,lon)；行 0=经度，行 1=纬度 |
| 视觉 / 渲染 | Real-ESRGAN x4plus + PyTorch CUDA | 背景瓦片 4× 超分（fp16，RTX 4080） |
| 视频后处理 | ffmpeg + libx264 + Pillow | H.264 编码，旋转后 center-crop |
| 数据分析 | NumPy + matplotlib | 延误对比柱状图；按方向 bucket 平均 |
| 检测数据来源 | LinkVision REST + WebSocket API | 抛锚车事件、ITS object 元数据 |
| **测试** | unittest | 5 个测试文件覆盖 events / signal_optimizer / sumo_replay / terasim_runner / main 的 calculate_delay |
| 工程封装 | Python argparse + bash run_*.sh | 用例编排（25-case sweep、5-plan compare、LinkVision replay） |

---

## 5. 整体流程详解

### 5.1 单次基础仿真流程（`main.py` → `SUMODelayCalculator.run`）

```
1. 解析参数（obstacles "lat,lon", mode, program_id, sim_time）
2. _load_network_projection() 读 net.xml 的 netOffset/origBoundary
3. traci.start(sumo-gui, numRetries=10)   # 启动 SUMO
4. (GUI) gui.setOffset / setZoom / setAngle  # 镜头对准路口
5. add_obstacles_via_traci()              # 注入静止 obstacle_veh
6. inject_manual_vehicles()               # 注入初始车队（main.py 里硬编码 ~30 辆）
7. set_tls_program_via_traci()            # 可选：JSON 自定义 phase
8. while step < total_steps:
     simulationStep()
     update_obstacle_positions()          # 强制 obstacle 位置 + speed=0
     update_tls_program()                 # 仅一次：bench/opt/dynamic 决策
                                          #   -> 同时调 _record_applied_tls(reason)
     trigger_rerouting(step*step_length)  # 30 秒（实时秒）周期
     assist_stuck_vehicles(t)             # 30/60/100s 阶梯换道
     remove_stuck_vehicles(t)             # waiting>180s → teleport
     collect_vehicle_data(t)              # 累积 waitingTime
9. calculate_delay() + save_results()
   # JSON 中 tls_program 字段记录的是 *applied_tls_program* (实际切到了哪个 program / 为什么切)
   # 而不是用户传入的 --tls-program 参数
```

### 5.2 LinkVision 真实事件接入流程（本分支新增）

```
LinkVision JSON 文件 (events + metadata)
    │
    ▼
load_replay_events()       # linkvision_terasim/sumo_replay.py
    │  filter event_name == "Unexpected Stop"
    │  filter detected_objects.warning == True
    │  pick max(confidence_score)
    │  bbox bottom-center → (px, py)
    │  homography → (lat, lon)
    │
    ▼
select_replay_event(event_id 或 latest)   # default: 时间戳最大
    │
    ▼
build_comparison_cases()   # 默认 modes=("bench","dynamic")
    │  生成 main.py 命令行（subprocess 路径）
    │  或 TeraSimReplayConfig（TeraSim 路径）
    │
    ▼
run_replay_cases() / run_terasim_replay()
    │
    ▼
delay_linkvision_<eventID>_<mode>.json + tripinfo + (TeraSim) fcd_all + run.log
```

### 5.3 影片级演示流程（`main_demo.py` → 7 阶段，未变）

`SUMODemoRunner` 继承 `SUMODelayCalculator`，把 run_simulation 改写为剧本式 7 阶段：

| Phase | 名称 | 关键动作 |
|---|---|---|
| 1 | **Start** | 用 `*_flowless.rou.xml`（去掉 flow 仅留 vType+route）启动 SUMO；镜头放在 overview 大全景 |
| 2 | **Place** | `_preload_edge_positions()` 用 ElementTree 直读 net.xml 建立点索引（比 TraCI 快约 60×）→ 注入 obstacle、初始车（manual_veh_*）、4000 背景车（bg_veh_*） |
| 3 | **Conflict check** | `_check_conflicts()` 把距离 manual/obstacle < 10m 的 bg 车删除，避免初始重叠 |
| 4 | **Pre-roll + Two-stage zoom-in** | 5s 预热（背景车跑、初始车冻结）→ Stage A 镜头 200→1200 zoom，Stage B 1200→10000 + 旋转 60° |
| 5 | **Pause + Release** | 释放 manual_veh：`setSpeedMode(31), setLaneChangeMode(1621), setSpeed(-1)` |
| 6 | **Start flows** | 解析原 rou.xml 的 flow 定义，按 `vehsPerHour` 自己用 `traci.vehicle.add` 在主循环里手撒 |
| 7 | **Main loop** | 每帧 simulationStep + screenshot；最后 ffmpeg 拼成 60fps mp4 |

### 5.4 多用例对比脚本

| 脚本 | 内容 |
|---|---|
| `run_cases.sh` | 3 用例：无障碍 / 障碍+原 TLS / 障碍+动态 TLS（脚本里大半 case 已被注释掉，当前只跑 1 个） |
| `run_plan_cases.sh` | 5 用例：org / plan_1..plan_4，固定 WB 抛锚 |
| `run_simulations.sh` | 25 用例：1 基线 + 12 静态 + 12 动态（4 方向 × 3 车道） |
| `run_full_new.sh` | 在大网络跑录像；先用 SUMO 自带 randomTrips.py 生成 ~3500 背景 trip，再合并 |
| `run_all.sh` / `run_overview.sh` | 串行跑 main_demo + main_overview |
| **`run_linkvision_replay.py`** | **真实事件 → main.py 子进程对比 bench vs dynamic** |
| **`run_linkvision_terasim.py`** | **真实事件 → TeraSim 框架内对比 bench vs dynamic** |

---

## 6. 核心函数与代码模块

### 6.1 `main.py — SUMODelayCalculator`（≈58k 字节，单文件）

| 方法 | 作用 |
|---|---|
| `latlon_to_xy(lat, lon)` | 通过 `traci.simulation.convertGeo(fromGeo=True)` 把 GPS 转 SUMO 局部 Mercator 坐标 |
| `_find_nearest_edge(x,y)` | O(N·点数) 遍历所有 edge.lane.shape，返回最近的 (edge_id, lane_idx)；演示版被 KD-cache 重写 |
| `add_obstacles_via_traci` | obstacle 实现：`vehicle.add → moveToXY(keepRoute=2) → setSpeedMode(0) → setSpeed(0) → setStop(duration=2³¹-1)`，并染红 `(255,0,0)` |
| `inject_manual_vehicles` / `_inject_single_vehicle` | 按 GPS 注入初始车队（main.py 末尾硬编码 30+ 辆），先 convertRoad→snap，失败 fallback 到 _find_nearest_edge |
| `update_obstacle_positions` | 每 step 强制 `moveToXY` + `setSpeed(0)` 防漂移 |
| `_get_through_lanes(edge_id)` | 用 `lane.getLinks()` 第 7 元素 `dir == 's'` 找出所有"直行车道"，并缓存 |
| `assist_stuck_vehicles` | 30/60/100s 阶梯：调 `setParameter("laneChangeModel.lcPushy")` 从 0.5 升到 1.0；100s 后 `setLaneChangeMode(0)` 强制换道，无视安全检查 |
| `trigger_rerouting(current_time)` | **每 30 秒（实时秒，非 step）** 扫描，对 `accumulatedWaitingTime>30 & speed<1` 的车调 `rerouteTraveltime`；本分支把口径从 step 改成秒 |
| `remove_stuck_vehicles` | `getWaitingTime ≥ 180s` 的非 obstacle 车直接 `vehicle.remove(reason=2)` (TELEPORT) |
| `update_tls_program` | 仅执行一次。bench→`setProgram(org)`，opt→`setProgram(opt)`，dynamic→把 obstacle 所在 lane_id 当 program ID，找不到则 fallback 到 `<edge>_*` 或 `opt`；**每个分支都调 `_record_applied_tls()`** 记录实际生效的 program 与 reason |
| `_record_applied_tls(tls_id, program_id, reason)` | **本分支新增**。把 (tls_id, applied_program_id, reason) 存到 `self.applied_tls_program`，最后写入输出 JSON 的 `configuration.tls_program` 字段 |
| `set_tls_program_via_traci` | 加载 JSON 配置（如 `tls_config_example.json`），构造 `traci.trafficlight.Logic(phases, programID, …)` 并 `setProgramLogic` 替换原 program |
| `collect_vehicle_data` / `calculate_delay` | 累积 `accumulatedWaitingTime` → 当 vehicle.arrived 时写入 arrival_time，最终求平均；**空到达时也补全 `total_*`/`simulation_time`/`total_departed` 字段**（本分支修） |
| `run_simulation` | 主循环；本分支 `traci.start(numRetries=10)`、`config_file` 改为相对当前文件的相对路径（去掉硬编码 `/home/yilinwang/...`） |

### 6.2 `main_demo.py — SUMODemoRunner`（≈61k 字节）

| 方法 | 作用 |
|---|---|
| `_compute_window_size` | 反推 SUMO 窗口尺寸：θ=60° 旋转后中心裁剪 3840×2160 不能黑边 → 加 15% 余量 |
| `smooth_camera_move` | 余弦缓动 ease-in-out，每帧 `setOffset/setZoom` + simulationStep |
| `_create_flowless_route_file` | 生成同名 `_flowless.rou.xml`：去 flow/trip，仅保留 vType + route，缓存复用 |
| `_parse_flow_definitions` | 解析原 rou.xml 的 flow，求出 `interval = 3600/vehsPerHour`，运行时手动 spawn |
| `_discover_downtown_edges` | 在 bbox `(-121.8985,37.3315) → (-121.8855,37.3395)` 内找所有 passenger 可走 edge |
| `_place_background_vehicles` | 按 `n_lanes × length` 加权分布 4000 辆，目的地随机；100m 内排除区避免堵到 obstacle |
| `_check_conflicts` / `_remove_background_vehicles` | manual/obstacle 周边 10m 内的 bg 车删掉；可指定 keep_center+radius 保留中心区 |
| `_step_and_capture` | `simulationStep` + 黑名单删除（`bg_veh_2315/2317`） + 重定向 (`bg_veh_2474→417034059#0` 等) + screenshot |
| `_compose_video` | 旋转 + center-crop + LANCZOS resize 用 PIL；ffmpeg `libx264 crf=18 yuv420p` |

### 6.3 `main_overview.py — SUMOOverviewRunner`
继承 `SUMODemoRunner`，关闭 zoom 动画与旋转，仅做"固定俯视镜头 30s"演示。本分支只是 trigger_rerouting 调用口径修复。

### 6.4 `main_scene2.py — SUMOScene2Runner`
另一路口（37.331966, -121.900180）+ 更宽 zoom=600，所有车一次性放好同步释放，含 scene2a / scene2b 两段。本分支只是 trigger_rerouting 调用口径修复。

### 6.5 网络 / 数据生成
- `merge_networks.py` — **add-only XML merge**：解析 downtown vs full 的 edge/junction/connection 集合差，仅插入 NEW 元素，绝不替换；保证版本兼容。
- `generate_12phase_traffic.py` — 给 12 条预定义 route（r_0..r_11）按 EW/NS 流量比例自动生成 flow。
- `generate_directional_routes.py` — 类似工具，可调 directional asymmetry。
- `upscale_tiles.py` — Real-ESRGAN 4× 超分；先 monkey-patch `torchvision.transforms.functional_tensor`（torchvision ≥0.18 已删除）兼容 basicsr。

### 6.6 信号方案设计（osm.tls.xml）
同一路口 ID `cluster_1984576776_3478559735_3478559736_3537422682_#1more` 下定义多个 program，各自 18 phase，100s 周期：

| programID | 含义 | EW 主绿 | NS 主绿 |
|---|---|---|---|
| `org` | 默认 | 26.4s | 27.8s |
| `plan_1` … `plan_3` | 中间档 | 26.5 → 28.2s | 19.0 → 17.3s |
| `1418903639#0_2` (plan_4 / opt) | WB 抛锚最优 | 28.2s | 16.5s |
| `<lane_id>`（如 `-416901230#1_3`）| 按车道命名，dynamic 模式直接匹配 | — | — |

### 6.7 `linkvision_terasim/`（本分支新增包，4 个模块）

> 设计要点：把"事件解析 / 信号决策 / 子进程编排 / TeraSim 集成"四件事拆开，每件是一个文件。`signal_optimizer` 是纯函数，`events` 是无副作用 dataclass —— 所以可以单测；带副作用的部分（subprocess、TeraSim）单独包起来。

#### `events.py` — LinkVision JSON → 数据类
| 函数 / 类 | 作用 |
|---|---|
| `CameraCalibration` | dataclass：`camera_id`, `resolution_w/h`, `homography_matrix(3x3 tuple)`, lat/lon, timezone |
| `RealWorldStalledVehicleEvent` | dataclass：源 event_id, task_id, timestamp, camera_id, object_id/name, confidence, bbox, pixel_x/y, lat/lon, image_url, video_url |
| `load_camera_calibrations(metadata_path)` | 读 `response_its_task_metadata.json` → `{camera_id: CameraCalibration}` |
| `normalized_bbox_to_pixel(bbox, W, H, anchor)` | LinkVision bbox 是归一化 [0,1] 的 (x,y,w,h)；默认 anchor=`bottom_center`（车体落地点）→ (px, py) |
| `apply_homography(H, px, py)` | **关键**：H[0]=经度行、H[1]=纬度行（LinkVision 约定），返回 `(lat, lon)` |
| `_select_warning_vehicle()` | 在 detected_objects 里挑 `warning=True` 且 `object_name ∈ {car, truck, bus, suv, pickup, van, motorcycle}` 的最高 confidence 的对象 |
| `iter_stalled_vehicle_events()` | 主管道：filter event_name=="Unexpected Stop" → 选目标车 → bbox → pixel → homography → 产出 `RealWorldStalledVehicleEvent` |
| `event_to_obstacle_arg(event)` | 拼成 main.py 期望的 "lat,lon" 字符串 |

#### `signal_optimizer.py` — 纯函数决策器
- 常量：`DEFAULT_TLS_ID`、`WB_OPTIMAL_PROGRAM_ID = "1418903639#0_2"`
- `SignalPlanDecision(tls_id, program_id, reason)` dataclass
- `choose_signal_plan(obstacle_lane_id, direction, available_programs, ...)` → `SignalPlanDecision`
  - 优先级：`exact_lane_program` > `edge_program`（按 `<edge>_*` 前缀匹配）> `directional_optimal_plan`（仅当 direction=="WB" 且 WB_OPTIMAL_PROGRAM_ID 在 available_programs 中）> `fallback_opt` > `fallback_default`
  - **这把 main.py 里 `update_tls_program` 的判分逻辑抽出来做了纯函数**，让 TeraSim 路径和测试都能复用

#### `sumo_replay.py` — main.py 子进程编排
| 函数 | 作用 |
|---|---|
| `select_replay_event(events, event_id)` | 给定 event_id 取那一个，否则取**时间戳最大**（最新）那个 |
| `load_replay_events(events_json, metadata_json)` | 串起 `load_camera_calibrations` + `iter_stalled_vehicle_events` |
| `build_sumo_command(...)` | 拼 `python main.py --net-file ... --route-file ... --obstacles "lat,lon" --mode bench|dynamic --output ... --sim-time ... --gui|--no-gui` |
| `build_comparison_cases(event, ..., modes=("bench","dynamic"))` | 默认产 bench + dynamic 两个 `ReplayCase`，输出名 `delay_linkvision_<eventID>_<mode>.json` |
| `run_replay_cases(cases)` | `subprocess.run(check=True)` 执行所有命令 |

#### `terasim_runner.py` — TeraSim 框架版
| 组件 | 作用 |
|---|---|
| `DEFAULT_TERASIM_HOME = /Users/happyfewmac/Desktop/Inchor/Terasim` | 硬编码默认路径（CLI 可覆盖）|
| `TeraSimReplayConfig` | 启动配置：terasim_home / sumo_net_file / sumo_config_file / sumo_route_file / output_path / mode / gui / sim_time / step_length |
| `add_terasim_to_path(home)` | 运行时把 `<home>/packages/terasim` 插到 `sys.path`；不存在则抛 |
| `prepare_sumolib_compatible_net_file(net_file, output_dir)` | **解决 bug：** TeraSim 通过 sumolib withPrograms=True 读 net 时，期望 phase duration 是整数；本网络里有 `26.40` 这种小数。这里把所有 `<phase duration="...">` 四舍五入成 int 写到一份副本里，**只供 TeraSim 内部 net 对象使用**——SUMO 跑的时候依然加载原 net.xml |
| `LinkVisionReplayEnv(EnvTemplate)` | TeraSim 环境模板。`on_start`：注入 stalled vehicle + apply 信号方案；`on_step`：每步 pin 住 stalled vehicle，到 `sim_time` 返回 False 结束 |
| `_add_stalled_vehicle()` | 复制 vehicletype `DEFAULT_VEHTYPE → linkvision_stalled_vehicle`；`convertGeo → convertRoad` 找到 (edge, lane_pos, lane_idx)；`route.add` + `vehicle.add(depart=0, departSpeed=0)` + `setSpeedMode(0)/setLaneChangeMode(0)` + 染红 |
| `_apply_signal_plan()` | bench → `org`；dynamic → `getAllProgramLogics` 拿 available_programs，丢给 `choose_signal_plan()`，再 `setProgram` |
| `_pin_stalled_vehicle()` | 每步 `setSpeed(0) + moveTo(lane, lane_position)`，对抗 SUMO 内部把车推走 |
| `run_terasim_replay(event, config)` | 准备 net 副本 → 起 `Simulator(sumo_output_file_types=["tripinfo","fcd_all"])` → bind env → run |

### 6.8 顶层 CLI 入口（本分支新增）

| 文件 | 作用 |
|---|---|
| `run_linkvision_replay.py` | 默认从 `traffic_data_analysis/linkVision_rawData/{response_stalled_car_detected.json, response_its_task_metadata.json}` 读事件；`--mode {bench,dynamic,compare}`；`--dry-run` 只打印命令；输出到 `traffic_data_analysis/delay_result/delay_linkvision_<id>_<mode>.json` |
| `run_linkvision_terasim.py` | 同上选事件，但走 TeraSim 框架；`--terasim-home` 覆盖默认；输出到 `outputs/linkvision_terasim/linkvision_<id>_<mode>/` |
| `analyze_tripinfo_by_direction.py` | 读 `tripinfo_1521072_{bench,dynamic}.xml`，按 `flow_<corridor>_<dir>_<turn>` 把 vehicle id 拆成 (corridor, direction)；同时按"该路径是否经过 BLOCKED_LANE_EDGE=`-416901230#1`"分桶，对 bench/dynamic 输出 duration / timeLoss / waiting 三组对比表。**用来回答"动态切灯到底是普惠提升还是抢了反向时间补给堵向？"** |
| `inspect_tls_programs.py` | 直接 `xml.etree` 解析 `osm.net.xml`（绕过 sumolib 的 float 问题），列出所有 `tlLogic.id` + `programID`，并校验 `DEFAULT_TLS_ID`、`WB_OPTIMAL_PROGRAM_ID`、`opt`、`org` 都存在。Smoke test |

### 6.9 测试（`tests/`，本分支新增）

| 文件 | 覆盖内容 |
|---|---|
| `test_linkvision_events.py` | bbox→pixel(`bottom_center`)、homography 投影（用真实 metadata，断言事件 1521072 → lat 37.33538562 / lon −121.89221894）、camera_id 缺失被跳过、最小 metadata 文件加载 |
| `test_signal_optimizer.py` | 4 个 case：exact_lane / edge_program fallback / WB direction → optimal_plan / unknown → opt |
| `test_sumo_replay.py` | select_replay_event 优先级（event_id → latest）；build_comparison_cases 拼出 bench+dynamic 两条命令、含 --obstacles + --mode + --sim-time |
| `test_terasim_runner.py` | prepare_sumolib_compatible_net_file 把 `26.40` → `26`（int 化） |
| `test_main_results.py` | calculate_delay 在 0 到达时也返回完整 schema |

---

## 7. 关键结果

### 7.1 合成场景（README 的旧表，仍代表项目设计意图）

| 场景 | Avg Delay | 到达车辆 | 解读 |
|---|---|---|---|
| 默认计划，无障碍 | 26.1s | 767 | 基线 |
| 默认计划 + WB 抛锚 | 31.8s (+22%) | 739 | 损失明显 |
| Plan 1 (EW 26.5s) | 29.9s | 760 | 略改善 |
| Plan 4 / Optimal (EW 28.2s) | 28.3s (−11% vs 默认+障碍) | 764 | **最优**：通行量恢复，延误几乎回到无障碍水平 |

### 7.2 真实 LinkVision 事件（event id = 1521072，本分支新增）

| 文件 | 模式 | applied program / reason | avg_delay | total_arrived / departed | total_wait_time |
|---|---|---|---|---|---|
| `delay_linkvision_1521072_bench.json` | bench | `org` / bench_mode_default | **30.79s** | 772 / 819 | 23 768s |
| `delay_linkvision_1521072_dynamic.json` | dynamic | `-416901230#1_3` / **exact_lane_program** | **27.74s** | 762 / 809 | 21 136s |

**解读**：dynamic mode 把 `update_tls_program` 决策落到了 `exact_lane_program` 分支——也就是说 osm.tls.xml 里恰好有一份名字等于这条受堵 lane 的 program。这也佐证了 `signal_optimizer.choose_signal_plan` 的**优先级 0**（lane 名字直接当 program ID）在真实事件下是命中的，无需 fallback。

> ⚠️ 真实事件的延误数字（30.79s）大于"默认 + WB 抛锚"合成场景（31.8s 接近）；这是因为不同事件的注入位置和 routes 集会略有差异，应当按 (bench, dynamic) 同事件**配对比较**，而不是跨事件比较绝对值。

### 7.3 按方向拆向（`analyze_tripinfo_by_direction.py`）

源代码注释里写：
> dynamic mode showed +11s avg trip duration despite −3s avg delay. Hypothesis: signal plan favors blocked direction (EW east), so the opposite direction (EW west) trades wait time for longer trips.

也就是说，dynamic 切灯虽然总平均延误↓，但**平均行程时长↑**——很可能是把 NS 方向的"红灯额度"挪给了 EW 方向，让 NS 车多等。这是 GTC 演讲里需要诚实交代的 trade-off。脚本会按 (corridor_dir, blocked_relation, full_flow_id) 三种粒度打 bench vs dynamic 对比表，便于在演示前把这个细节量化清楚。

---

## 8. 技术名词词典（Glossary）

| 名词 | 解释 |
|---|---|
| **SUMO** | Eclipse 基金会开源微观交通仿真器，处理车辆-道路-信号的离散步进物理 |
| **TraCI** | "Traffic Control Interface"，SUMO 暴露的 TCP API；Python 端 `import traci` 后可在仿真任意时刻读写状态 |
| **TeraSim** | SaferDrive-AI 维护的仿真框架，把 SUMO+TraCI 包成 `Simulator/EnvTemplate/VehicleFactory` 的 OO 模板，本项目把它放在 `/Users/happyfewmac/Desktop/Inchor/Terasim` 下作为外部依赖 |
| **net.xml** | SUMO 路网文件，定义 edge / lane / junction / connection / TLS |
| **rou.xml / trips.xml** | 路径与车辆需求文件；`<route>` 是固定边序列，`<flow>` 是按 vehsPerHour 持续放车，`<trip>` 是 from/to 由 SUMO 自找路径 |
| **TLS / tlLogic** | Traffic Light Signal；`<tlLogic>` 内 `<phase duration state>`，state 字符串里每个字符对应一条 controlled lane（G/g 绿、y 黄、r 红） |
| **programID** | 同一 TLS 可挂多套 program；运行时 `traci.trafficlight.setProgram(tls_id, programID)` 切换 |
| **phase** | 一个固定时长的灯色组合；多个 phase 按顺序构成一个 cycle |
| **actuated TLS** | 感应式信号——当某 phase 检测到 jam 超阈值（`tls.actuated.jam-threshold=30`）时自动延长 |
| **applied_tls_program** | 本项目自定义结果字段：`{tls_id, applied_program_id, reason}`，记录 dynamic 决策**实际落到了哪一档**（exact_lane / edge / opt fallback / no_obstacle 等）|
| **homography_matrix** | 相机平面 ↔ 地面 GPS 平面的 3×3 单应矩阵；摄像头检测到的 (px,py) 通过它映射到 (lat,lon)。LinkVision 约定第 1 行=经度，第 2 行=纬度 |
| **bottom_center anchor** | 把 bbox 的下边中点作为车辆"落地像素"，是把 2D 检测投影到地面的标准做法 |
| **moveToXY(keepRoute=2)** | 在 TraCI 里把车强行 teleport 到 (x,y)；keepRoute=2 表示忽略原 route，可放在任意 edge |
| **convertGeo / convertRoad** | TraCI：经纬度 ↔ SUMO xy；xy ↔ (edge_id, lane_pos, lane_idx) |
| **rerouteTraveltime** | 用当前 SUMO 内部边权重（含拥堵）重算最短路径 |
| **device.rerouting.period=30** | 每 30s 自动 reroute；`adaptation-steps=18` 表示用 18 步指数滑动平均算 edge weight |
| **Krauss model** | 经典跟驰模型，安全距离 + 期望速度反馈，参数 τ（反应时间）、σ（驾驶员误差）|
| **SL2015** | "sublane 2015"，SUMO 的 sub-lane 横向模型，能处理"骑线"、并排行驶；参数 lcPushy / lcAssertive / lcImpatience 控制激进度 |
| **lcPushy / lcAssertive / lcImpatience** | 强迫前车让位 / 容忍小间隙 / 等待越久越激进 |
| **vTypeDistribution** | 多个 vType 按概率随机抽样的"车型混合分布"；本项目 `realistic_traffic_mix` 内含 13 种 |
| **GPS → SUMO 投影** | net.xml 头部记录 `netOffset`、`projParameter`；TraCI 提供 `convertGeo(lon,lat,fromGeo=True)` 一行完成 |
| **fcd_all** | TeraSim 输出选项：每步导出所有车辆 (x,y,speed) 的 floating-car data，可用于离线轨迹回放 / 可视化 |
| **Real-ESRGAN / RRDBNet** | Tencent ARC 开源盲超分网络（基于残差密集块）；basicsr 是依赖工具集，新版 torchvision 删了 `functional_tensor`，需 monkey-patch |
| **fp16 / half** | 半精度浮点；RTX 4080 上推理 ~2× 提速 |
| **ffmpeg libx264 crf=18 yuv420p** | H.264 软件编码，crf 越小越无损（18 视觉无损）；4:2:0 色度采样兼容浏览器 / QuickTime |
| **center-crop after rotation** | 旋转后边角"黑三角"，按 `aspect·cosθ + sinθ` 反推安全裁剪框可避免 |
| **LinkVision** | LinkerVision 的视觉感知 / ITS API；本项目从 staging 抓了"抛锚车检测"事件回包 + 摄像头 metadata |
| **stalled vehicle** | 在交通工程语境下指无法移动的事故 / 抛锚车，是 *non-recurring congestion* 的主要源头 |
| **spillback** | 排队溢出，下游路口阻塞往上游传播 |
| **dynamic / actuated / fixed-time control** | 三种信号控制范式：固定配时 / 感应式 / 动态自适应（按事件切方案） |

---

## 9. 风险 / 盲点 / 改进点（Risk & Gap Analysis）

| 维度 | 当前现状 | 潜在问题 / 改进建议 |
|---|---|---|
| **delay 计算口径** | `average_delay = total_waitingTime / arrived_count` | 严格意义的 *delay* 是 `actual_time − ideal_time`（即 `timeLoss`），代码注释也承认"using waiting time as approximation"。建议改用 SUMO `tripinfo.timeLoss`，本分支已在 `analyze_tripinfo_by_direction.py` 里读 timeLoss 列了，可以把口径在 main.py 里也对齐 |
| **TLS 切换只跑一次** | `update_tls_program` 内 `_tls_program_applied` 闸门确保只切一次 | 若同一 TLS 周期内有多个抛锚事件先后发生（LinkVision 也支持事件流推送），需要把闸门去掉并按事件流重算。`signal_optimizer.choose_signal_plan` 的纯函数设计已经为此铺路 |
| **`_find_nearest_edge` O(N·点数)** | main.py 每个 obstacle 一次完整扫描 | 已在 main_demo 重写为预加载 list；对几千 edge 仍是 O(N)。可换成 KD-Tree |
| **dynamic 程序匹配** | 程序 ID 必须等于 obstacle 所在 lane_id（`exact_lane_program`） | 命名硬约束，路网更新就坏。`signal_optimizer` 已加了 `edge_program` 与 `directional_optimal_plan`（WB→`1418903639#0_2`）两层 fallback，进一步稳健 |
| **assist_stuck_vehicles 强制换道** | 100s 后 `setLaneChangeMode(0) + changeLane(target,15s)` 忽略安全检查 | 极端情况下可能"穿模"，仅用于演示和量化，不要直接套到真车控制 |
| **routes 版本不一致** | downtown vs full 的 r_1…r_12 边序列不同（直行 / 左转拓扑略改）| 部分 case 结果差异可能来自这里。建议加 schema test |
| **LinkVision token 写在仓库 1.txt** | `traffic_data_analysis/linkVision_rawData/1.txt` 含 staging JWT | **安全风险**：JWT 应删除 / 作废 / 移出仓库（`.gitignore` 当前是否覆盖待确认）|
| **TeraSim home 硬编码** | `terasim_runner.DEFAULT_TERASIM_HOME = /Users/happyfewmac/Desktop/Inchor/Terasim` | 换机器就坏；CLI `--terasim-home` 已可覆盖，建议在 README 里强调，必要时改读环境变量 `TERASIM_HOME` |
| **sumolib float-duration workaround** | `prepare_sumolib_compatible_net_file` 把 `26.40 → 26` | 只改 sumolib 那份副本是临时方案，TeraSim 上游修复后可移除 |
| **dynamic 模式的 NS↔EW 取舍** | 全局 avg_delay 下降，但**单方向 trip duration 上升** | 演讲时需要诚实陈述（`analyze_tripinfo_by_direction.py` 已有量化），不要只展示总平均数 |
| **video composition 大内存** | PIL 解码 ~3840×2160 PNG + bicubic rotate | 长片段需要分块；可改用 ffmpeg 内置 `rotate=PI/3:ow=...` 滤镜直接做 |

---

## 10. 结论与下一步建议

**结论**：San-Jose-GTC-Demo 是一个工程完整度很高的 SUMO 数字孪生 + AI 视觉融合 demo。本分支的关键升级是**把"合成的 lat,lon 字符串注入"升级为"消费 LinkVision 真实事件 JSON"**，并且把决策逻辑从 main.py 抽出来做成纯函数 + unittest，新增了一条可选的 TeraSim 框架托管路径。在真实事件 1521072 上得到了 −9.9% avg_delay 的正向结果，足以支撑 GTC 演讲中"sensor → twin → control"故事的可信度。

**Recommended Action**（按优先级）：
1. **去敏感**：`traffic_data_analysis/linkVision_rawData/1.txt` 里的 staging JWT 移出仓库并轮换；
2. **delay 口径统一**：把 `calculate_delay` 改用 `tripinfo.timeLoss`（甚至直接解析 `<statisticOutput>`），与 SUMO 官方一致；
3. **打开事件流**：解除 `update_tls_program` 的 `_tls_program_applied` 单次锁，结合 `signal_optimizer.choose_signal_plan` 的纯函数设计，做成"每个新事件都重决策"；
4. **routes 对齐**：让 downtown 与 full 的 r_1..r_12 完全一致，避免对比误差；
5. **演讲数据诚实**：演讲时除了总 avg_delay，把 `analyze_tripinfo_by_direction.py` 的"按方向 trade-off"输出也带上一张图；
6. **CI 化**：把 `tests/`（unittest）和 `run_simulations.sh` 25 用例做成 GitHub Actions matrix，每次 commit 自动出对比图。

---

## Citations / Sources

- README.md（项目根目录）
- LINKVISION_TERASIM.md（本分支新增的子说明）
- main.py / main_demo.py / main_overview.py / main_scene2.py（核心代码）
- linkvision_terasim/{events.py, signal_optimizer.py, sumo_replay.py, terasim_runner.py}（本分支新增包）
- run_linkvision_replay.py / run_linkvision_terasim.py / analyze_tripinfo_by_direction.py / inspect_tls_programs.py（本分支新增脚本）
- tests/test_{linkvision_events, signal_optimizer, sumo_replay, terasim_runner, main_results}.py
- merge_networks.py、generate_12phase_traffic.py、upscale_tiles.py
- san_jose_downtown_gtc/osm.tls.xml（多 program TLS 定义）
- san_jose_full_new/intersection_flows.rou.xml（vTypeDistribution + flow）
- traffic_data_analysis/linkVision_rawData/{response_its_task_metadata.json, response_stalled_car_detected.json, ite_inte.txt, ite_object.txt}
- traffic_data_analysis/delay_result/delay_linkvision_1521072_{bench,dynamic}.json（真实事件实测）
- traffic_data_analysis/tripinfo/tripinfo_1521072_{bench,dynamic}.xml（按方向分析的输入）
- outputs/linkvision_terasim/linkvision_1521072_{bench,dynamic}/{tripinfo.xml, fcd_all.xml, run.log}（TeraSim 路径产物）
- 外部参考：[SUMO Docs](https://sumo.dlr.de/docs/) · [TraCI Docs](https://sumo.dlr.de/docs/TraCI.html) · [Real-ESRGAN GitHub](https://github.com/xinntao/Real-ESRGAN) · [TeraSim](https://github.com/SaferDrive-AI/TeraSim)

# Delay Analysis: Static vs Dynamic TLS with Stalled Vehicles

## Simulation Setup

- **Network**: San Jose Downtown (osm.net.xml)
- **Routes**: directional_traffic.rou.xml
- **Simulation time**: 3600 seconds
- **Total runs**: 25 (1 benchmark + 12 static + 12 dynamic)
- **Obstacle locations**: 12 approach lanes (EB/WB/NB/SB x left/thr/right)

---

## 1. Benchmark (No Obstacles)

| Metric | Value |
|--------|-------|
| Average delay | **13.17 s** |
| Average trip duration | 564.75 s |
| Vehicles arrived | 56 / 61 (91.8%) |
| Total time loss | 737.5 s |

---

## 2. Average Delay Comparison (seconds)

| Location | Static | Dynamic | Improvement | Dynamic vs Benchmark |
|----------|-------:|--------:|------------:|---------------------:|
| **EB_left** | 18.90 | 21.40 | -13.2% (worse) | +62.5% |
| **EB_thr** | 17.04 | 12.45 | **+26.9%** | -5.4% |
| **EB_right** | 19.61 | 15.63 | **+20.3%** | +18.7% |
| **WB_left** | 16.51 | 11.85 | **+28.2%** | -10.0% |
| **WB_thr** | 15.96 | 11.93 | **+25.2%** | -9.4% |
| **WB_right** | 16.25 | 11.81 | **+27.3%** | -10.3% |
| NB_left | 15.02 | 15.02 | 0.0% | +14.0% |
| NB_thr | 14.69 | 14.69 | 0.0% | +11.5% |
| NB_right | 14.68 | 14.68 | 0.0% | +11.5% |
| SB_left | 15.30 | 15.30 | 0.0% | +16.2% |
| SB_thr | 14.67 | 14.67 | 0.0% | +11.4% |
| SB_right | 14.61 | 14.61 | 0.0% | +10.9% |

---

## 3. Total Time Loss Comparison (seconds)

| Location | Static | Dynamic | Reduction |
|----------|-------:|--------:|----------:|
| **EB_left** | 1020.7 | 1134.0 | -11.1% (worse) |
| **EB_thr** | 937.3 | 684.9 | **+26.9%** |
| **EB_right** | 1098.3 | 875.4 | **+20.3%** |
| **WB_left** | 908.1 | 663.5 | **+26.9%** |
| **WB_thr** | 893.5 | 680.1 | **+23.9%** |
| **WB_right** | 910.0 | 673.4 | **+26.0%** |
| NB_left | 856.0 | 856.0 | 0.0% |
| NB_thr | 837.4 | 837.4 | 0.0% |
| NB_right | 837.0 | 837.0 | 0.0% |
| SB_left | 872.0 | 872.0 | 0.0% |
| SB_thr | 836.2 | 836.2 | 0.0% |
| SB_right | 832.8 | 832.8 | 0.0% |

---

## 4. Completion Rate (Arrived / Departed)

| Location | Benchmark | Static | Dynamic |
|----------|----------:|-------:|--------:|
| (none) | 56/61 (91.8%) | -- | -- |
| EB_left | -- | 54/62 (87.1%) | 53/62 (85.5%) |
| EB_thr | -- | 55/62 (88.7%) | 55/62 (88.7%) |
| EB_right | -- | 56/62 (90.3%) | 56/62 (90.3%) |
| WB_left | -- | 55/62 (88.7%) | 56/62 (90.3%) |
| WB_thr | -- | 56/62 (90.3%) | 57/62 (91.9%) |
| WB_right | -- | 56/62 (90.3%) | 57/62 (91.9%) |
| NB_left | -- | 57/62 (91.9%) | 57/62 (91.9%) |
| NB_thr | -- | 57/62 (91.9%) | 57/62 (91.9%) |
| NB_right | -- | 57/62 (91.9%) | 57/62 (91.9%) |
| SB_left | -- | 57/62 (91.9%) | 57/62 (91.9%) |
| SB_thr | -- | 57/62 (91.9%) | 57/62 (91.9%) |
| SB_right | -- | 57/62 (91.9%) | 57/62 (91.9%) |

---

## 5. Key Findings

### 5.1 Eastbound (EB) and Westbound (WB) obstacles are the most disruptive
- Static mode delays increase **+21% to +49%** over benchmark for EB/WB lanes
- EB_right (static) has the worst average delay at **19.61 s** (+48.9% over benchmark)
- NB/SB obstacles have smaller impact (+10% to +16% over benchmark)

### 5.2 Dynamic TLS significantly reduces delay for most EB/WB cases
- **WB lanes benefit the most**: dynamic mode reduces delay by **25-28%** compared to static, achieving delays **below the benchmark** (10% lower than no-obstacle scenario)
- **EB_thr**: dynamic cuts delay by **26.9%** (17.04 -> 12.45 s), also ending up below benchmark
- **EB_right**: dynamic cuts delay by **20.3%** (19.61 -> 15.63 s)

### 5.3 EB_left is the exception -- dynamic mode performs worse
- EB_left is the only case where dynamic TLS **increases** delay (+13.2%, from 18.90 to 21.40 s)
- Also has the lowest completion rate in the entire study (53/62 = 85.5%)
- This suggests the dynamic TLS program for the EB_left lane may need tuning

### 5.4 NB and SB directions are unaffected by TLS mode switching
- All NB and SB results are **identical** between static and dynamic modes
- This indicates the dynamic TLS program does not alter phase timing for NB/SB approaches
- The obstacle on these lanes has a moderate but consistent impact (+11-16% delay)

### 5.5 Dynamic TLS can outperform no-obstacle baseline
- For WB_left, WB_thr, WB_right, and EB_thr under dynamic mode, the average delay is **lower than the benchmark** (no obstacles at all)
- This suggests the dynamic TLS program not only compensates for the blocked lane but also improves overall signal efficiency for those phases

---

## 6. Summary by Direction

| Direction | Avg Static Delay | Avg Dynamic Delay | Avg Improvement |
|-----------|-----------------:|------------------:|----------------:|
| EB | 18.52 s | 16.49 s | +11.0% |
| WB | 16.24 s | 11.86 s | **+27.0%** |
| NB | 14.80 s | 14.80 s | 0.0% |
| SB | 14.86 s | 14.86 s | 0.0% |
| **Benchmark** | **13.17 s** | -- | -- |

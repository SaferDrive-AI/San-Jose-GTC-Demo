#!/usr/bin/env python3
"""Bucket tripinfo trips by direction and compare bench vs dynamic.

Why: dynamic mode showed +11s avg trip duration despite -3s avg delay.
Hypothesis: signal plan favors blocked direction (EW east), so the opposite
direction (EW west) trades wait time for longer trips.

Each vehicle id from a SUMO flow looks like 'flow_ew_east_left.42'.
We split off the flow prefix and map it to (corridor, direction, turn).
"""
from __future__ import annotations

import statistics
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

PROJECT = Path(__file__).resolve().parent
TRIPINFO_DIR = PROJECT / "traffic_data_analysis/tripinfo"
ROUTE_FILE = PROJECT / "san_jose_downtown_gtc/directional_traffic.rou.xml"
BLOCKED_LANE_EDGE = "-416901230#1"
REVERSE_LANE_EDGE = "416901230#1"


def load_flow_to_route() -> dict[str, str]:
    tree = ET.parse(ROUTE_FILE)
    return {f.get("id"): f.get("route") for f in tree.getroot().findall("flow")}


def load_route_to_edges() -> dict[str, list[str]]:
    tree = ET.parse(ROUTE_FILE)
    return {r.get("id"): r.get("edges", "").split() for r in tree.getroot().findall("route")}


def classify(flow_id: str, edges: list[str]) -> tuple[str, str]:
    """Returns (corridor_direction, blocked_relation)."""
    parts = flow_id.split("_")  # flow ew east left
    corridor_dir = f"{parts[1]}_{parts[2]}" if len(parts) >= 3 else "unknown"
    if BLOCKED_LANE_EDGE in edges:
        rel = "passes_blocked"
    elif REVERSE_LANE_EDGE in edges:
        rel = "passes_reverse"
    else:
        rel = "unrelated"
    return corridor_dir, rel


def parse_tripinfo(path: Path) -> list[dict]:
    rows = []
    for ti in ET.parse(path).getroot().findall("tripinfo"):
        rows.append(
            {
                "id": ti.get("id", ""),
                "duration": float(ti.get("duration", 0)),
                "timeLoss": float(ti.get("timeLoss", 0)),
                "waitingTime": float(ti.get("waitingTime", 0)),
                "routeLength": float(ti.get("routeLength", 0)),
            }
        )
    return rows


def aggregate(rows: list[dict], flow_to_route: dict[str, str], route_to_edges: dict[str, list[str]]):
    by_dir = defaultdict(lambda: {"duration": [], "timeLoss": [], "waiting": [], "len": []})
    by_rel = defaultdict(lambda: {"duration": [], "timeLoss": [], "waiting": [], "len": []})
    by_flow = defaultdict(lambda: {"duration": [], "timeLoss": [], "waiting": [], "len": []})

    for r in rows:
        flow_id = r["id"].rsplit(".", 1)[0]
        route_id = flow_to_route.get(flow_id)
        edges = route_to_edges.get(route_id, [])
        corridor_dir, rel = classify(flow_id, edges)
        for bucket, key in [(by_dir, corridor_dir), (by_rel, rel), (by_flow, flow_id)]:
            bucket[key]["duration"].append(r["duration"])
            bucket[key]["timeLoss"].append(r["timeLoss"])
            bucket[key]["waiting"].append(r["waitingTime"])
            bucket[key]["len"].append(r["routeLength"])
    return by_dir, by_rel, by_flow


def fmt_avg(samples: list[float]) -> str:
    return f"{statistics.mean(samples):7.2f}" if samples else "    n/a"


def print_table(title: str, bench_buckets, dynamic_buckets):
    print(f"\n=== {title} ===")
    keys = sorted(set(bench_buckets) | set(dynamic_buckets))
    print(
        f"{'bucket':<25} {'n_b':>5} {'n_d':>5} "
        f"{'dur_b':>8} {'dur_d':>8} {'Δdur':>8}  "
        f"{'loss_b':>8} {'loss_d':>8} {'Δloss':>8}  "
        f"{'wait_b':>8} {'wait_d':>8} {'Δwait':>8}"
    )
    print("-" * 130)
    for k in keys:
        b = bench_buckets.get(k, {"duration": [], "timeLoss": [], "waiting": []})
        d = dynamic_buckets.get(k, {"duration": [], "timeLoss": [], "waiting": []})
        nb, nd = len(b["duration"]), len(d["duration"])
        if not nb or not nd:
            continue
        dur_b, dur_d = statistics.mean(b["duration"]), statistics.mean(d["duration"])
        loss_b, loss_d = statistics.mean(b["timeLoss"]), statistics.mean(d["timeLoss"])
        w_b, w_d = statistics.mean(b["waiting"]), statistics.mean(d["waiting"])
        print(
            f"{k:<25} {nb:>5d} {nd:>5d} "
            f"{dur_b:>8.2f} {dur_d:>8.2f} {dur_d-dur_b:>+8.2f}  "
            f"{loss_b:>8.2f} {loss_d:>8.2f} {loss_d-loss_b:>+8.2f}  "
            f"{w_b:>8.2f} {w_d:>8.2f} {w_d-w_b:>+8.2f}"
        )


def main() -> int:
    flow_to_route = load_flow_to_route()
    route_to_edges = load_route_to_edges()

    bench_path = TRIPINFO_DIR / "tripinfo_1521072_bench.xml"
    dynamic_path = TRIPINFO_DIR / "tripinfo_1521072_dynamic.xml"
    if not bench_path.exists() or not dynamic_path.exists():
        print(f"missing tripinfo files. expected:\n  {bench_path}\n  {dynamic_path}")
        return 1

    bench_rows = parse_tripinfo(bench_path)
    dyn_rows = parse_tripinfo(dynamic_path)
    print(f"bench tripinfo entries: {len(bench_rows)}")
    print(f"dynamic tripinfo entries: {len(dyn_rows)}")

    by_dir_b, by_rel_b, by_flow_b = aggregate(bench_rows, flow_to_route, route_to_edges)
    by_dir_d, by_rel_d, by_flow_d = aggregate(dyn_rows, flow_to_route, route_to_edges)

    print_table("By relation to blocked lane", by_rel_b, by_rel_d)
    print_table("By corridor direction", by_dir_b, by_dir_d)
    print_table("By flow (full split)", by_flow_b, by_flow_d)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

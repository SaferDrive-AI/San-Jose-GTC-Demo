#!/usr/bin/env python
from __future__ import annotations
"""Build deterministic SUMO flows from VSS intersection counts.

Pipeline stages:
1) map VSS lat/lon intersections to SUMO nodes/approaches,
2) remove internal duplicate inflows and keep boundary injection roots,
3) expand deterministic turn patterns into fixed edge routes,
4) emit SUMO route/flow XML and run quick simulation sanity checks.
"""

import argparse
import json
import math
import subprocess
import xml.etree.ElementTree as ET
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import sumolib

CARDINALS = ("EB", "WB", "NB", "SB")
TURNS = ("left", "through", "right")


@dataclass
class Pattern:
    pattern_id: str
    edges: list[str]
    vehs_per_hour: float
    decisions: list[tuple[str, str, str]]  # (intersection_id, incoming_dir, turn)


def heading_deg(dx: float, dy: float) -> float:
    return (math.degrees(math.atan2(dx, dy)) + 360.0) % 360.0


def nearest_cardinal(heading: float) -> str:
    axes = [(0, "NB"), (90, "EB"), (180, "SB"), (270, "WB")]
    return min(axes, key=lambda t: min((heading - t[0]) % 360, (t[0] - heading) % 360))[1]


def signed_turn(in_h: float, out_h: float) -> float:
    return ((out_h - in_h + 540.0) % 360.0) - 180.0


def classify_turn_by_angle(angle: float) -> str:
    if angle > 35:
        return "left"
    if angle < -35:
        return "right"
    return "through"


def edge_heading_into_node(edge) -> float:
    shape = edge.getShape()
    if len(shape) >= 2:
        x1, y1 = shape[-2]
        x2, y2 = shape[-1]
    else:
        x1, y1 = edge.getFromNode().getCoord()
        x2, y2 = edge.getToNode().getCoord()
    return heading_deg(x2 - x1, y2 - y1)


def edge_heading_out_of_node(edge) -> float:
    shape = edge.getShape()
    if len(shape) >= 2:
        x1, y1 = shape[0]
        x2, y2 = shape[1]
    else:
        x1, y1 = edge.getFromNode().getCoord()
        x2, y2 = edge.getToNode().getCoord()
    return heading_deg(x2 - x1, y2 - y1)


def top_nodes(net, lon: float, lat: float, n: int = 5):
    x, y = net.convertLonLat2XY(lon, lat)
    rows = []
    for node in net.getNodes():
        if node.getID().startswith(":"):
            continue
        nx, ny = node.getCoord()
        d2 = (nx - x) ** 2 + (ny - y) ** 2
        rows.append((d2, node))
    rows.sort(key=lambda r: r[0])
    out = []
    for d2, node in rows[:n]:
        out.append({"node_id": node.getID(), "distance_m": round(math.sqrt(d2), 2)})
    return out


def build_approaches(net, node):
    incoming = [e for e in node.getIncoming() if not e.getID().startswith(":") and e.allows("passenger")]
    by_card = {k: [] for k in CARDINALS}
    for edge in incoming:
        h = edge_heading_into_node(edge)
        by_card[nearest_cardinal(h)].append((edge, h))

    approaches = {}
    for card, cands in by_card.items():
        if not cands:
            continue

        def score(item):
            e, _ = item
            cnt = 0
            for out_e, conns in e.getOutgoing().items():
                if out_e.getID().startswith(":"):
                    continue
                if conns:
                    cnt += 1
            return cnt

        edge, h = max(cands, key=score)
        approaches[card] = {"edge_id": edge.getID(), "heading": h, "movements": {}}

    for card, info in approaches.items():
        in_edge = net.getEdge(info["edge_id"])
        in_h = info["heading"]
        grouped = {t: [] for t in TURNS}
        for out_edge, conns in in_edge.getOutgoing().items():
            if not conns or out_edge.getID().startswith(":"):
                continue
            if out_edge.getToNode().getID() == in_edge.getFromNode().getID():
                continue

            out_h = edge_heading_out_of_node(out_edge)
            angle = signed_turn(in_h, out_h)
            # For input turn schema (left/through/right), exclude explicit U-turn connectors.
            valid_conns = [c for c in conns if c.getDirection() != "t"]
            if not valid_conns:
                continue
            dirs = {c.getDirection() for c in valid_conns}
            if "r" in dirs:
                turn = "right"
            elif "l" in dirs:
                turn = "left"
            elif "s" in dirs:
                turn = "through"
            else:
                turn = classify_turn_by_angle(angle)
            grouped[turn].append({"to_edge": out_edge.getID(), "angle": angle, "abs_angle": abs(angle)})

        for t in TURNS:
            cands = grouped[t]
            if not cands:
                continue
            if t == "through":
                best = min(cands, key=lambda c: c["abs_angle"])
            elif t == "left":
                best = max(cands, key=lambda c: c["angle"])
            else:
                best = min(cands, key=lambda c: c["angle"])
            info["movements"][t] = best["to_edge"]

    return approaches


def map_intersections(net, data, max_dist_m: float):
    results = []
    for inter in data.get("intersections", []):
        iid = inter["intersection_id"]
        lat = float(inter["lat"])
        lon = float(inter["lon"])
        counts = inter.get("counts", {})

        cands = top_nodes(net, lon, lat, n=5)
        entry = {
            "intersection_id": iid,
            "lat": lat,
            "lon": lon,
            "counts": counts,
            "candidate_nodes": cands,
            "status": "unmatched",
            "reason": None,
            "matched_node_id": None,
            "distance_to_node_m": None,
            "approaches": {},
        }
        if not cands:
            entry["reason"] = "no_candidate_node"
            results.append(entry)
            continue

        best = cands[0]
        node = net.getNode(best["node_id"])
        dist = float(best["distance_m"])
        approaches = build_approaches(net, node)

        entry["matched_node_id"] = node.getID()
        entry["distance_to_node_m"] = dist
        entry["approaches"] = approaches
        if dist > max_dist_m:
            entry["reason"] = f"distance_too_far>{max_dist_m}m"
        else:
            entry["status"] = "matched"
            entry["reason"] = "ok"
        results.append(entry)
    return results


def _bearing_between(a_xy, b_xy) -> float:
    ax, ay = a_xy
    bx, by = b_xy
    return heading_deg(bx - ax, by - ay)


def _angle_diff(a: float, b: float) -> float:
    return min((a - b) % 360.0, (b - a) % 360.0)


def _min_hops_from_sources_to_target(net, source_edges: set[str], target_edge: str, max_hops: int = 12):
    q = deque()
    visited = set()
    for s in source_edges:
        q.append((s, 0))
        visited.add(s)
    while q:
        eid, hops = q.popleft()
        if eid == target_edge:
            return hops
        if hops >= max_hops:
            continue
        edge = net.getEdge(eid)
        for out_edge, conns in edge.getOutgoing().items():
            oid = out_edge.getID()
            if oid.startswith(":") or not conns:
                continue
            if oid in visited:
                continue
            visited.add(oid)
            q.append((oid, hops + 1))
    return None


def identify_internal_inflows(net, mapped_results, angle_thresh_deg: float = 55.0, max_dist_m: float = 700.0, max_hops: int = 10):
    matched = [r for r in mapped_results if r["status"] == "matched"]
    by_id = {r["intersection_id"]: r for r in matched}
    source_edges = {r["intersection_id"]: {info["edge_id"] for info in r["approaches"].values()} for r in matched}
    internal = {r["intersection_id"]: set() for r in matched}

    for r in matched:
        iid = r["intersection_id"]
        ixy = net.getNode(r["matched_node_id"]).getCoord()
        for d, info in r["approaches"].items():
            target_edge = info["edge_id"]
            source_bearing = (info["heading"] + 180.0) % 360.0

            candidates = []
            for oid, other in by_id.items():
                if oid == iid:
                    continue
                oxy = net.getNode(other["matched_node_id"]).getCoord()
                dist = math.dist(ixy, oxy)
                if dist > max_dist_m:
                    continue
                b = _bearing_between(ixy, oxy)
                ad = _angle_diff(source_bearing, b)
                if ad <= angle_thresh_deg:
                    candidates.append((oid, dist, ad))
            candidates.sort(key=lambda x: (x[1], x[2]))

            for oid, _dist, _ad in candidates:
                hops = _min_hops_from_sources_to_target(net, source_edges[oid], target_edge, max_hops=max_hops)
                if hops is not None:
                    internal[iid].add(d)
                    break

    return internal


def build_boundary_json(net, original, mapped_results):
    internal_dirs = identify_internal_inflows(net, mapped_results)
    mapped_by_id = {r["intersection_id"]: r for r in mapped_results}

    out = {
        "schema_version": original.get("schema_version", "1.0"),
        "corridor_id": original.get("corridor_id"),
        "observation_interval": original.get("observation_interval"),
        "bucket_seconds": original.get("bucket_seconds"),
        "window": original.get("window"),
        "intersections": [],
        "injection_strategy": {
            "type": "auto_boundary_from_mapped_graph",
            "notes": "Keep only boundary inflow directions; internal linked inflows are removed.",
            "removed_internal_directions": {},
            "dropped_unmapped_or_unusable_directions": {},
        },
    }

    for inter in original.get("intersections", []):
        iid = inter["intersection_id"]
        counts = inter.get("counts", {})
        mapped = mapped_by_id.get(iid)
        kept = {}
        removed_internal = []
        dropped_unusable = []

        if not mapped or mapped["status"] != "matched":
            out["injection_strategy"]["dropped_unmapped_or_unusable_directions"][iid] = list(counts.keys())
            out["intersections"].append({"intersection_id": iid, "lat": inter["lat"], "lon": inter["lon"], "counts": {}})
            continue

        approach_dirs = set(mapped["approaches"].keys())
        for d, turn_counts in counts.items():
            if d not in approach_dirs:
                dropped_unusable.append(d)
                continue
            if d in internal_dirs.get(iid, set()):
                removed_internal.append(d)
                continue
            moves = mapped["approaches"][d]["movements"]
            cleaned = {t: float(turn_counts.get(t, 0.0)) for t in TURNS if t in moves and float(turn_counts.get(t, 0.0)) > 0}
            if cleaned:
                kept[d] = cleaned

        if removed_internal:
            out["injection_strategy"]["removed_internal_directions"][iid] = removed_internal
        if dropped_unusable:
            out["injection_strategy"]["dropped_unmapped_or_unusable_directions"][iid] = dropped_unusable

        out["intersections"].append({"intersection_id": iid, "lat": inter["lat"], "lon": inter["lon"], "counts": kept})

    return out


def _next_edges(net, edge_id: str, allow_uturn: bool):
    edge = net.getEdge(edge_id)
    out = []
    for oe, conns in edge.getOutgoing().items():
        oid = oe.getID()
        if oid.startswith(":") or not conns:
            continue
        if not oe.allows("passenger"):
            continue
        # Follow map semantics: do not traverse explicit U-turn connectors.
        valid_conns = [c for c in conns if c.getFromLane().allows("passenger") and c.getToLane().allows("passenger")]
        if not valid_conns:
            continue
        dirs = {c.getDirection() for c in valid_conns}
        if (not allow_uturn) and ("t" in dirs):
            continue
        # Also avoid immediate reverse-edge bounce (eid -> -eid).
        if edge_id.lstrip("-") == oid.lstrip("-") and edge_id != oid:
            continue
        out.append(oid)
    return out


def _bfs_path(start_edge: str, cond: Callable[[str], bool], net, max_hops: int, allow_uturn: bool):
    q = deque([(start_edge, [start_edge], 0)])
    visited = {start_edge}
    while q:
        eid, path, hops = q.popleft()
        if cond(eid):
            return path
        if hops >= max_hops:
            continue
        for ne in _next_edges(net, eid, allow_uturn=allow_uturn):
            if ne in visited:
                continue
            visited.add(ne)
            q.append((ne, path + [ne], hops + 1))
    return None


def _pick_straightest_outgoing(edge_id: str, net, allow_uturn: bool):
    curr = net.getEdge(edge_id)
    curr_h = edge_heading_out_of_node(curr)
    best = None
    best_abs = float("inf")
    for ne in _next_edges(net, edge_id, allow_uturn=allow_uturn):
        e2 = net.getEdge(ne)
        out_h = edge_heading_out_of_node(e2)
        d = abs(signed_turn(curr_h, out_h))
        if d < best_abs:
            best_abs = d
            best = ne
    return best


def _walk_straightest(start_edge: str, net, hops: int, allow_uturn: bool):
    route = [start_edge]
    seen = {start_edge}
    cur = start_edge
    for _ in range(hops):
        ne = _pick_straightest_outgoing(cur, net, allow_uturn=allow_uturn)
        if not ne or ne in seen:
            break
        route.append(ne)
        seen.add(ne)
        cur = ne
    return route


def _ensure_min_route_length(route_edges: list[str], net, min_edges: int, allow_uturn: bool, extra_hops: int = 10):
    if len(route_edges) >= min_edges:
        return route_edges
    out = list(route_edges)
    seen = set(out)
    cur = out[-1]
    for _ in range(extra_hops):
        if len(out) >= min_edges:
            break
        ne = _pick_straightest_outgoing(cur, net, allow_uturn=allow_uturn)
        if not ne or ne in seen:
            break
        out.append(ne)
        seen.add(ne)
        cur = ne
    return out


def _sanitize_connected_route(route_edges: list[str], net):
    if not route_edges:
        return route_edges
    out = [route_edges[0]]
    for nxt in route_edges[1:]:
        cur = out[-1]
        ok = False
        edge = net.getEdge(cur)
        for oe, conns in edge.getOutgoing().items():
            if oe.getID() == nxt and conns:
                valid_conns = [c for c in conns if c.getFromLane().allows("passenger") and c.getToLane().allows("passenger")]
                if not valid_conns:
                    continue
                ok = True
                break
        if not ok:
            break
        out.append(nxt)
    return out


def _extend_route_upstream(route_edges: list[str], net, upstream_hops: int):
    """Prepend a few upstream edges so vehicles spawn farther from the intersection."""
    if upstream_hops <= 0 or not route_edges:
        return route_edges
    first = route_edges[0]
    first_base = first.lstrip("-")
    out = list(route_edges)
    seen = set(out)
    cur = net.getEdge(first)
    prefix = []

    def can_feed_all_passenger_lanes(up_edge, down_edge_id: str) -> bool:
        passenger_from = set()
        for li in range(up_edge.getLaneNumber()):
            ln = up_edge.getLane(li)
            if ln.allows("passenger"):
                passenger_from.add(ln.getID())
        if not passenger_from:
            return False

        fed = set()
        for oe, conns in up_edge.getOutgoing().items():
            if oe.getID() != down_edge_id:
                continue
            for c in conns:
                fl = c.getFromLane()
                tl = c.getToLane()
                if fl.allows("passenger") and tl.allows("passenger"):
                    fed.add(fl.getID())
        # Require every passenger lane to have at least one legal continuation.
        return passenger_from.issubset(fed)
    for _ in range(upstream_hops):
        incomings = []
        for ie in cur.getIncoming():
            iid = ie.getID()
            if iid.startswith(":"):
                continue
            if not ie.allows("passenger"):
                continue
            if iid in seen:
                continue
            # Do not prepend the immediate reverse of the route start edge.
            # This avoids spawn points jumping across the intersection side.
            if iid.lstrip("-") == first_base and iid != first:
                continue
            if not can_feed_all_passenger_lanes(ie, cur.getID()):
                continue
            incomings.append(ie)
        if not incomings:
            break
        # Prefer the longest upstream edge to push spawn point visibly farther.
        best = max(incomings, key=lambda e: e.getLength())
        prefix.append(best.getID())
        seen.add(best.getID())
        cur = best
    prefix.reverse()
    return prefix + out


def _turn_weights(counts_for_dir: dict, available_turns: set[str]):
    raw = {t: float(counts_for_dir.get(t, 0.0)) for t in available_turns}
    s = sum(raw.values())
    if s <= 0:
        if not available_turns:
            return {}
        w = 1.0 / len(available_turns)
        return {t: w for t in available_turns}
    return {t: v / s for t, v in raw.items()}


def enumerate_patterns(
    net,
    iid: str,
    incoming_dir: str,
    flow_vehph: float,
    route_prefix: list[str],
    decisions_prefix: list[tuple[str, str, str]],
    visited_iids: set[str],
    mapped_by_id: dict,
    counts_by_iid: dict,
    approach_owner_by_edge: dict,
    max_corridor_hops: int,
    exit_hops: int,
    allow_uturn: bool,
):
    if flow_vehph <= 1e-6:
        return []

    mapped = mapped_by_id[iid]
    app = mapped["approaches"].get(incoming_dir)
    if not app:
        # no approach, terminate current prefix as one pattern
        pid = " > ".join([f"{a}.{b}.{c}" for a, b, c in decisions_prefix]) or f"{iid}.{incoming_dir}.sink"
        return [Pattern(pid, route_prefix, flow_vehph, decisions_prefix)]

    move_map = app["movements"]
    available_turns = {t for t in TURNS if t in move_map}
    if not available_turns:
        pid = " > ".join([f"{a}.{b}.{c}" for a, b, c in decisions_prefix]) or f"{iid}.{incoming_dir}.sink"
        return [Pattern(pid, route_prefix, flow_vehph, decisions_prefix)]

    counts_for_dir = counts_by_iid.get(iid, {}).get(incoming_dir, {})
    weights = _turn_weights(counts_for_dir, available_turns)

    patterns: list[Pattern] = []
    for turn, w in weights.items():
        child_flow = flow_vehph * w
        if child_flow <= 1e-6:
            continue
        first_edge = move_map[turn]

        # Find next VSS approach edge forward.
        def cond(edge_id: str):
            if edge_id not in approach_owner_by_edge:
                return False
            nxt_iid, _nxt_dir = approach_owner_by_edge[edge_id]
            return nxt_iid != iid

        path = _bfs_path(first_edge, cond, net, max_hops=max_corridor_hops, allow_uturn=allow_uturn)
        decision_chain = decisions_prefix + [(iid, incoming_dir, turn)]

        if path is None:
            tail = _walk_straightest(first_edge, net, hops=exit_hops, allow_uturn=allow_uturn)
            new_route = route_prefix + ([tail[0]] if not route_prefix or route_prefix[-1] != tail[0] else []) + tail[1:]
            pid = " > ".join([f"{a}.{b}.{c}" for a, b, c in decision_chain])
            patterns.append(Pattern(pid, new_route, child_flow, decision_chain))
            continue

        # Append path to next VSS approach.
        if route_prefix and route_prefix[-1] == path[0]:
            new_route = route_prefix + path[1:]
        else:
            new_route = route_prefix + path

        nxt_edge = path[-1]
        nxt_iid, nxt_dir = approach_owner_by_edge[nxt_edge]
        if nxt_iid in visited_iids:
            tail = _walk_straightest(nxt_edge, net, hops=exit_hops, allow_uturn=allow_uturn)
            if new_route and new_route[-1] == tail[0]:
                new_route = new_route + tail[1:]
            else:
                new_route = new_route + tail
            pid = " > ".join([f"{a}.{b}.{c}" for a, b, c in decision_chain]) + " > loop_stop"
            patterns.append(Pattern(pid, new_route, child_flow, decision_chain))
            continue

        patterns.extend(
            enumerate_patterns(
                net=net,
                iid=nxt_iid,
                incoming_dir=nxt_dir,
                flow_vehph=child_flow,
                route_prefix=new_route,
                decisions_prefix=decision_chain,
                visited_iids=visited_iids | {nxt_iid},
                mapped_by_id=mapped_by_id,
                counts_by_iid=counts_by_iid,
                approach_owner_by_edge=approach_owner_by_edge,
                max_corridor_hops=max_corridor_hops,
                exit_hops=exit_hops,
                allow_uturn=allow_uturn,
            )
        )

    return patterns


def build_patterns(net, boundary_json, mapped_results, max_corridor_hops: int, exit_hops: int, allow_uturn: bool):
    mapped_by_id = {r["intersection_id"]: r for r in mapped_results if r["status"] == "matched"}
    counts_by_iid = {i["intersection_id"]: i.get("counts", {}) for i in boundary_json.get("intersections", [])}

    approach_owner_by_edge = {}
    for iid, m in mapped_by_id.items():
        for d, info in m["approaches"].items():
            approach_owner_by_edge[info["edge_id"]] = (iid, d)

    patterns: list[Pattern] = []
    for inter in boundary_json.get("intersections", []):
        iid = inter["intersection_id"]
        if iid not in mapped_by_id:
            continue
        for d, turn_counts in inter.get("counts", {}).items():
            if d not in mapped_by_id[iid]["approaches"]:
                continue
            inflow = float(sum(float(turn_counts.get(t, 0.0)) for t in TURNS))
            if inflow <= 0:
                continue
            from_edge = mapped_by_id[iid]["approaches"][d]["edge_id"]
            patterns.extend(
                enumerate_patterns(
                    net=net,
                    iid=iid,
                    incoming_dir=d,
                    flow_vehph=inflow,
                    route_prefix=[from_edge],
                    decisions_prefix=[],
                    visited_iids={iid},
                    mapped_by_id=mapped_by_id,
                    counts_by_iid=counts_by_iid,
                    approach_owner_by_edge=approach_owner_by_edge,
                    max_corridor_hops=max_corridor_hops,
                    exit_hops=exit_hops,
                    allow_uturn=allow_uturn,
                )
            )

    # Merge identical edge routes to reduce route count.
    merged: dict[tuple[str, ...], Pattern] = {}
    for p in patterns:
        key = tuple(p.edges)
        if key in merged:
            merged[key].vehs_per_hour += p.vehs_per_hour
        else:
            merged[key] = Pattern(p.pattern_id, list(p.edges), p.vehs_per_hour, list(p.decisions))
    return list(merged.values())


def write_deterministic_routes(
    route_out: Path,
    patterns: list[Pattern],
    net,
    sim_end: int,
    flow_scale: float,
    min_route_edges: int,
    allow_uturn: bool,
    spawn_upstream_hops: int,
):
    root = ET.Element("routes")
    root.set("xmlns:xsi", "http://www.w3.org/2001/XMLSchema-instance")
    root.set("xsi:noNamespaceSchemaLocation", "http://sumo.dlr.de/xsd/routes_file.xsd")
    ET.SubElement(root, "vType", {
        "id": "car_normal",
        "vClass": "passenger",
        "accel": "2.6",
        "decel": "4.5",
        "maxSpeed": "33.3",
        "sigma": "0.5",
        "tau": "1.2",
        "length": "4.5",
        "minGap": "2.5",
    })

    flow_count = 0
    start_edge_counts: dict[str, int] = {}
    for i, p in enumerate(patterns):
        scaled = p.vehs_per_hour * flow_scale
        if scaled <= 0.01:
            continue
        rid = f"route_{i}"
        fid = f"flow_{i}"
        edges = _sanitize_connected_route(p.edges, net)
        edges = _extend_route_upstream(edges, net, upstream_hops=spawn_upstream_hops)
        edges = _sanitize_connected_route(edges, net)
        edges = _ensure_min_route_length(edges, net, min_edges=min_route_edges, allow_uturn=allow_uturn)
        edges = _sanitize_connected_route(edges, net)
        if len(edges) < 2:
            continue
        ET.SubElement(root, "route", {"id": rid, "edges": " ".join(edges)})
        start_edge_counts[edges[0]] = start_edge_counts.get(edges[0], 0) + 1
        ET.SubElement(root, "flow", {
            "id": fid,
            "type": "car_normal",
            "begin": "0",
            "end": str(sim_end),
            "vehsPerHour": f"{scaled:.2f}",
            "route": rid,
            "departLane": "best",
            "departSpeed": "max",
        })
        flow_count += 1

    route_out.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(route_out, encoding="utf-8", xml_declaration=True)
    return flow_count, start_edge_counts


def run_sumo(net_file: Path, tls_file: Path, route_file: Path, sim_end: int, tripinfo_out: Path, fcd_out: Path, stats_out: Path):
    cmd = [
        "sumo",
        "-n",
        str(net_file),
        "-a",
        str(tls_file),
        "-r",
        str(route_file),
        "--begin",
        "0",
        "--end",
        str(sim_end),
        "--step-length",
        "1",
        "--no-step-log",
        "true",
        "--duration-log.statistics",
        "true",
        "--tripinfo-output",
        str(tripinfo_out),
        "--fcd-output",
        str(fcd_out),
        "--statistic-output",
        str(stats_out),
    ]
    return subprocess.run(cmd, check=False, capture_output=True, text=True)


def analyze_vehicle_motion(fcd_path: Path, tripinfo_path: Path, stop_speed: float, stop_threshold_s: int):
    arrived = set()
    if tripinfo_path.exists():
        tr = ET.parse(tripinfo_path).getroot()
        for t in tr.findall("tripinfo"):
            arrived.add(t.get("id"))

    per = {}
    # per[vid] = dict(last_t, last_stop, cur_stop, max_stop, seen_steps)
    if not fcd_path.exists():
        return {
            "vehicle_seen": 0,
            "arrived": 0,
            "long_stop_vehicle_count": 0,
            "long_stop_vehicle_ids": [],
            "max_stop_seconds": 0,
        }

    for event, elem in ET.iterparse(fcd_path, events=("start", "end")):
        if event == "end" and elem.tag == "timestep":
            t = float(elem.attrib.get("time", "0"))
            for v in elem.findall("vehicle"):
                vid = v.get("id")
                speed = float(v.get("speed", "0"))
                st = per.setdefault(vid, {"last_t": t, "cur_stop": 0.0, "max_stop": 0.0, "seen_steps": 0})
                dt = max(0.0, t - st["last_t"])
                st["last_t"] = t
                st["seen_steps"] += 1
                if speed <= stop_speed:
                    st["cur_stop"] += dt
                    if st["cur_stop"] > st["max_stop"]:
                        st["max_stop"] = st["cur_stop"]
                else:
                    st["cur_stop"] = 0.0
            elem.clear()

    long_ids = [vid for vid, st in per.items() if st["max_stop"] >= stop_threshold_s]
    max_stop = max((st["max_stop"] for st in per.values()), default=0.0)

    return {
        "vehicle_seen": len(per),
        "arrived": len(arrived),
        "long_stop_vehicle_count": len(long_ids),
        "long_stop_vehicle_ids": sorted(long_ids)[:50],
        "max_stop_seconds": round(max_stop, 2),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input-json", required=True)
    ap.add_argument("--net-file", required=True)
    ap.add_argument("--tls-file", required=True)
    ap.add_argument("--boundary-json-out", required=True)
    ap.add_argument("--route-out", required=True)
    ap.add_argument("--report-out", required=True)
    ap.add_argument("--patterns-out", required=True)
    ap.add_argument("--tripinfo-out", required=True)
    ap.add_argument("--fcd-out", required=True)
    ap.add_argument("--statistic-out", required=True)
    ap.add_argument("--sim-end", type=int, default=1800)
    ap.add_argument("--max-match-distance-m", type=float, default=60.0)
    ap.add_argument("--max-corridor-hops", type=int, default=12)
    ap.add_argument("--exit-hops", type=int, default=3)
    ap.add_argument("--flow-scale", type=float, default=0.35)
    ap.add_argument("--min-route-edges", type=int, default=5)
    ap.add_argument("--spawn-upstream-hops", type=int, default=2)
    ap.add_argument("--allow-uturn", action="store_true", default=True)
    ap.add_argument("--no-allow-uturn", action="store_false", dest="allow_uturn")
    ap.add_argument("--stop-speed", type=float, default=0.1)
    ap.add_argument("--stop-threshold-s", type=int, default=120)
    args = ap.parse_args()

    original = json.loads(Path(args.input_json).read_text(encoding="utf-8"))
    net = sumolib.net.readNet(args.net_file, withInternal=True)

    mapped = map_intersections(net, original, max_dist_m=args.max_match_distance_m)
    boundary_json = build_boundary_json(net, original, mapped)
    Path(args.boundary_json_out).write_text(json.dumps(boundary_json, indent=2), encoding="utf-8")

    patterns = build_patterns(
        net=net,
        boundary_json=boundary_json,
        mapped_results=mapped,
        max_corridor_hops=args.max_corridor_hops,
        exit_hops=args.exit_hops,
        allow_uturn=args.allow_uturn,
    )
    Path(args.patterns_out).write_text(
        json.dumps([
            {
                "pattern_id": p.pattern_id,
                "edges": p.edges,
                "vehs_per_hour": round(p.vehs_per_hour, 4),
                "decisions": p.decisions,
            }
            for p in patterns
        ], indent=2),
        encoding="utf-8",
    )

    flow_count, start_edge_counts = write_deterministic_routes(
        Path(args.route_out),
        patterns,
        net=net,
        sim_end=args.sim_end,
        flow_scale=args.flow_scale,
        min_route_edges=args.min_route_edges,
        allow_uturn=args.allow_uturn,
        spawn_upstream_hops=args.spawn_upstream_hops,
    )

    tripinfo_out = Path(args.tripinfo_out)
    fcd_out = Path(args.fcd_out)
    stats_out = Path(args.statistic_out)
    tripinfo_out.parent.mkdir(parents=True, exist_ok=True)
    fcd_out.parent.mkdir(parents=True, exist_ok=True)
    stats_out.parent.mkdir(parents=True, exist_ok=True)

    sim = run_sumo(
        net_file=Path(args.net_file),
        tls_file=Path(args.tls_file),
        route_file=Path(args.route_out),
        sim_end=args.sim_end,
        tripinfo_out=tripinfo_out,
        fcd_out=fcd_out,
        stats_out=stats_out,
    )

    motion = analyze_vehicle_motion(fcd_out, tripinfo_out, stop_speed=args.stop_speed, stop_threshold_s=args.stop_threshold_s)

    report = {
        "input_json": args.input_json,
        "flow_scale": args.flow_scale,
        "sim_end": args.sim_end,
        "mapped_intersections": mapped,
        "boundary_json_out": args.boundary_json_out,
        "patterns_out": args.patterns_out,
        "route_out": args.route_out,
        "pattern_count": len(patterns),
        "flow_count": flow_count,
        "spawn_start_edge_counts": start_edge_counts,
        "sumo_return_code": sim.returncode,
        "allow_uturn": args.allow_uturn,
        "sumo_stdout_tail": sim.stdout.splitlines()[-30:],
        "sumo_stderr_tail": sim.stderr.splitlines()[-30:],
        "vehicle_motion": motion,
    }
    Path(args.report_out).write_text(json.dumps(report, indent=2), encoding="utf-8")

    print(json.dumps({
        "boundary_json_out": args.boundary_json_out,
        "patterns_out": args.patterns_out,
        "route_out": args.route_out,
        "report_out": args.report_out,
        "pattern_count": len(patterns),
        "flow_count": flow_count,
        "sumo_return_code": sim.returncode,
        "vehicle_seen": motion["vehicle_seen"],
        "arrived": motion["arrived"],
        "long_stop_vehicle_count": motion["long_stop_vehicle_count"],
    }, indent=2))


if __name__ == "__main__":
    main()

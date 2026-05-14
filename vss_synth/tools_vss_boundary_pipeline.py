#!/usr/bin/env python
"""VSS → SUMO traffic synthesis pipeline (deterministic pattern enumeration).

Inputs:  VSS-like JSON (per-intersection TMC) + SUMO .net.xml + .tls.xml
Outputs: boundary JSON + .rou.xml (deterministic routes) + SUMO run report

Pipeline:
  1. map_intersections          — VSS lat/lon → SUMO junction match
  2. build_approaches           — incoming edges → EB/WB/NB/SB + per-turn outgoing
  3. identify_internal_inflows  — geometry + topology to find internal vs boundary
  4. build_boundary_json        — bookkeeping: which directions are boundary entry
  5. enumerate_patterns         — recursively expand turn-ratio tree into fixed paths
  6. write_deterministic_routes — emit one <route>+<flow> per leaf pattern
  7. run_sumo                   — sanity check the synthesized traffic

Why pattern enumeration (vs jtrrouter):
  jtrrouter samples turn ratios stochastically at every junction (VSS-known
  AND unknown), so each car's path is a chain of independent dice rolls. In a
  dense urban grid with default `--turn-defaults 25,50,25` at non-VSS
  junctions, this routinely produces routes that wander and loop.
  Pattern enumeration pre-computes, from the VSS turn ratios alone, a finite
  tree of (turn sequence → fixed edge path → fixed cars/h) tuples. Each
  outgoing turn at every VSS intersection splits the parent's car count by the
  local ratio; "through" branches that reach the next VSS intersection keep
  recursing; left/right branches that exit the corridor walk a couple more
  edges and terminate. At simulation time SUMO follows fixed routes — no
  randomness, no loops, traffic counts are exactly conserved at every VSS
  intersection by construction.
"""
from __future__ import annotations

import argparse
import json
import math
import subprocess
import xml.etree.ElementTree as ET
from collections import defaultdict, deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable

import sumolib

CARDINALS = ("EB", "WB", "NB", "SB")
TURNS = ("left", "through", "right")


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def heading_deg(dx: float, dy: float) -> float:
    return (math.degrees(math.atan2(dx, dy)) + 360.0) % 360.0


def nearest_cardinal(heading: float) -> str:
    axes = [(0, "NB"), (90, "EB"), (180, "SB"), (270, "WB")]
    return min(axes, key=lambda t: min((heading - t[0]) % 360, (t[0] - heading) % 360))[1]


def signed_turn(in_h: float, out_h: float) -> float:
    return ((out_h - in_h + 540.0) % 360.0) - 180.0


def classify_turn(angle: float) -> str:
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


# ---------------------------------------------------------------------------
# Intersection mapping (VSS lat/lon → SUMO node + approach edges)
# ---------------------------------------------------------------------------

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
    incoming = [
        e for e in node.getIncoming()
        if not e.getID().startswith(":") and e.allows("passenger")
    ]
    by_card: dict[str, list] = {k: [] for k in CARDINALS}
    for edge in incoming:
        h = edge_heading_into_node(edge)
        by_card[nearest_cardinal(h)].append((edge, h))

    approaches: dict[str, dict] = {}
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
        grouped: dict[str, list] = {t: [] for t in TURNS}
        for out_edge, conns in in_edge.getOutgoing().items():
            if not conns or out_edge.getID().startswith(":"):
                continue
            if out_edge.getToNode().getID() == in_edge.getFromNode().getID():
                continue  # immediate U-turn
            out_h = edge_heading_out_of_node(out_edge)
            angle = signed_turn(in_h, out_h)
            dirs = {c.getDirection() for c in conns}
            if "r" in dirs:
                turn = "right"
            elif "l" in dirs:
                turn = "left"
            elif "s" in dirs:
                turn = "through"
            else:
                turn = classify_turn(angle)
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


# ---------------------------------------------------------------------------
# Boundary detection (geometry + topology)
# ---------------------------------------------------------------------------

def _bearing_between(a_xy, b_xy) -> float:
    ax, ay = a_xy
    bx, by = b_xy
    return heading_deg(bx - ax, by - ay)


def _angle_diff(a: float, b: float) -> float:
    return min((a - b) % 360.0, (b - a) % 360.0)


def _min_hops_from_sources_to_target(net, source_edges: set[str], target_edge: str, max_hops: int = 12):
    q: deque = deque()
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


def identify_internal_inflows(
    net,
    mapped_results,
    angle_thresh_deg: float = 55.0,
    max_dist_m: float = 700.0,
    max_hops: int = 10,
):
    """Identify internal inflows by combining geometry and topological reachability.

    For each incoming direction at intersection I:
      - compute its upstream source bearing (heading + 180)
      - find another mapped intersection J that lies near this source bearing
      - verify J can reach this incoming edge within limited graph hops
    """
    matched = [r for r in mapped_results if r["status"] == "matched"]
    by_id = {r["intersection_id"]: r for r in matched}

    source_edges = {
        r["intersection_id"]: {info["edge_id"] for info in r["approaches"].values()}
        for r in matched
    }

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


def build_boundary_json(net, original, mapped_results, internal_dirs):
    mapped_by_id = {r["intersection_id"]: r for r in mapped_results}

    out = {
        "schema_version": original.get("schema_version", "1.0"),
        "corridor_id": original.get("corridor_id"),
        "observation_interval": original.get("observation_interval"),
        "intersections": [],
        "injection_strategy": {
            "type": "deterministic_pattern_enumeration",
            "notes": "Boundary directions are pattern-tree roots. Internal directions are visited downstream during recursion (no fresh injection there).",
            "boundary_directions": {},
            "internal_directions": {},
            "dropped_unmapped_or_unusable_directions": {},
        },
    }

    for inter in original.get("intersections", []):
        iid = inter["intersection_id"]
        counts = inter.get("counts", {})
        mapped = mapped_by_id.get(iid)
        boundary_dirs = []
        internal_at_iid = sorted(internal_dirs.get(iid, set())) if mapped and mapped["status"] == "matched" else []
        dropped_unusable = []

        if not mapped or mapped["status"] != "matched":
            out["injection_strategy"]["dropped_unmapped_or_unusable_directions"][iid] = list(counts.keys())
            out["intersections"].append({
                "intersection_id": iid,
                "lat": inter["lat"],
                "lon": inter["lon"],
                "boundary_directions": [],
                "internal_directions": [],
            })
            continue

        approach_dirs = set(mapped["approaches"].keys())
        for d in counts:
            if d not in approach_dirs:
                dropped_unusable.append(d)
                continue
            if d in internal_at_iid:
                continue
            turn_counts = counts[d]
            if any(float(v) > 0 for v in turn_counts.values()):
                boundary_dirs.append(d)

        if boundary_dirs:
            out["injection_strategy"]["boundary_directions"][iid] = boundary_dirs
        if internal_at_iid:
            out["injection_strategy"]["internal_directions"][iid] = internal_at_iid
        if dropped_unusable:
            out["injection_strategy"]["dropped_unmapped_or_unusable_directions"][iid] = dropped_unusable

        out["intersections"].append({
            "intersection_id": iid,
            "lat": inter["lat"],
            "lon": inter["lon"],
            "boundary_directions": boundary_dirs,
            "internal_directions": internal_at_iid,
        })

    return out


# ---------------------------------------------------------------------------
# Pattern enumeration — the deterministic core
# ---------------------------------------------------------------------------

@dataclass
class Pattern:
    """A deterministic flow pattern: a fixed edge sequence and a fixed car rate.

    Each pattern represents one branch through the VSS turn-ratio tree —
    e.g., "enter at market.EB, go through, then at 1st left-turn, then exit".
    The car rate is the joint probability along that branch multiplied by the
    boundary inflow rate.
    """
    pattern_id: str
    edges: list = field(default_factory=list)
    vehs_per_hour: float = 0.0
    decisions: list = field(default_factory=list)  # [(iid, direction, turn), ...]


def _pick_straightest_outgoing(net, current_edge):
    """Pick the next downstream edge with minimum heading change.

    Used for the exit tail. Reverse-direction (U-turn) edges are excluded —
    at a dead-end the only outgoing is often the reverse, and following it
    produces visually wrong cars that spin around mid-corridor.
    """
    curr_h = edge_heading_out_of_node(current_edge)
    curr_from = current_edge.getFromNode().getID()
    best = None
    best_abs = float("inf")
    for out_edge, conns in current_edge.getOutgoing().items():
        if not conns or out_edge.getID().startswith(":"):
            continue
        if not out_edge.allows("passenger"):
            continue
        if out_edge.getToNode().getID() == curr_from:
            continue
        out_h = edge_heading_out_of_node(out_edge)
        delta = abs(signed_turn(curr_h, out_h))
        if delta < best_abs:
            best_abs = delta
            best = out_edge
    return best


def _pick_straightest_incoming(net, current_edge):
    """Pick the predecessor edge that flows MOST-STRAIGHT into `current_edge`
    at their shared junction. Symmetric counterpart of straightest-outgoing.

    Used for the entry buffer: walking backward from a VSS approach to find a
    natural spawn point a couple of edges upstream. Reverse-direction edges
    (the immediate U-turn) are excluded.
    """
    from_node = current_edge.getFromNode()
    to_id = current_edge.getToNode().getID()
    curr_out_h = edge_heading_out_of_node(current_edge)
    best = None
    best_abs = float("inf")
    for prev_edge in from_node.getIncoming():
        pid = prev_edge.getID()
        if pid.startswith(":"):
            continue
        if not prev_edge.allows("passenger"):
            continue
        if prev_edge.getFromNode().getID() == to_id:
            continue  # U-turn: prev came from current's destination
        outgoing = prev_edge.getOutgoing()
        if current_edge not in outgoing or not outgoing[current_edge]:
            continue  # no SUMO connection prev → current
        prev_in_h = edge_heading_into_node(prev_edge)
        delta = abs(signed_turn(prev_in_h, curr_out_h))
        if delta < best_abs:
            best_abs = delta
            best = prev_edge
    return best


def _walk_forward_straightest(net, start_edge_id: str, hops: int) -> list:
    """Walk forward from start_edge for `hops` edges via the straightest
    heuristic. Returns list of edge IDs (excluding start). May return fewer
    than `hops` if the network dead-ends or loops back."""
    if hops <= 0:
        return []
    tail = []
    current = net.getEdge(start_edge_id)
    visited = {start_edge_id}
    for _ in range(hops):
        nxt = _pick_straightest_outgoing(net, current)
        if nxt is None:
            break
        nid = nxt.getID()
        if nid in visited:
            break
        tail.append(nid)
        visited.add(nid)
        current = nxt
    return tail


def _walk_backward_straightest(net, end_edge_id: str, hops: int) -> list:
    """Walk backward from end_edge for `hops` edges via the straightest
    heuristic. Returns the upstream edge chain in FORWARD order (i.e., the
    farthest-upstream edge first, end_edge last)."""
    chain = [end_edge_id]
    if hops <= 0:
        return chain
    current = net.getEdge(end_edge_id)
    visited = {end_edge_id}
    for _ in range(hops):
        prev = _pick_straightest_incoming(net, current)
        if prev is None:
            break
        pid = prev.getID()
        if pid in visited:
            break
        chain.insert(0, pid)
        visited.add(pid)
        current = prev
    return chain


def find_boundary_edges(net) -> set:
    """The set of network 'sink' edges — edges whose toNode has no valid
    forward (passenger-allowing, non-U-turn) outgoing edge.

    A vehicle whose route ENDS at such an edge naturally exits the simulation
    at the SUMO network's real perimeter (a true dead-end, or wherever the
    OSM extract was cut). Any vehicle that finishes its route at a
    non-boundary edge has effectively "vanished mid-network" — a routing
    bug.

    Implementation: passenger-allowing edges only (we never route bicycles /
    pedestrians here). 'No forward' means every outgoing from the toNode is
    either an internal edge, not passenger-allowing, or the reverse direction
    of the current edge (U-turn, disqualified).
    """
    boundary: set = set()
    for edge in net.getEdges():
        eid = edge.getID()
        if eid.startswith(":") or not edge.allows("passenger"):
            continue
        to_node = edge.getToNode()
        from_id = edge.getFromNode().getID()
        has_forward = False
        for out_edge in to_node.getOutgoing():
            oid = out_edge.getID()
            if oid.startswith(":") or not out_edge.allows("passenger"):
                continue
            if out_edge.getToNode().getID() == from_id:
                continue  # U-turn doesn't count as forward
            has_forward = True
            break
        if not has_forward:
            boundary.add(eid)
    return boundary


def _route_to_boundary(net, start_edge_id: str, boundary_edges: set, max_hops: int):
    """BFS forward from start_edge to the NEAREST network boundary edge.

    Returns:
        list of edge IDs (excluding the start edge) ending at the boundary
        edge — i.e., the corridor-exit tail that guarantees the vehicle
        leaves the simulation cleanly.
        None if no boundary is reachable within max_hops (signals a routing
        gap that the caller should handle).

    BFS visits edges in order of hop distance, so the first boundary edge it
    encounters is the closest forward exit. Reverse-direction (U-turn)
    edges are excluded so we never produce nonsense paths that loop back
    through prior intersections.
    """
    if start_edge_id in boundary_edges:
        return []  # start is already at a boundary

    q: deque = deque()
    q.append((start_edge_id, ()))
    visited = {start_edge_id}

    while q:
        eid, path = q.popleft()
        if len(path) >= max_hops:
            continue
        edge = net.getEdge(eid)
        edge_from = edge.getFromNode().getID()
        for out_edge, conns in edge.getOutgoing().items():
            oid = out_edge.getID()
            if oid.startswith(":") or not conns or oid in visited:
                continue
            if not out_edge.allows("passenger"):
                continue
            if out_edge.getToNode().getID() == edge_from:
                continue  # block U-turn
            new_path = path + (oid,)
            if oid in boundary_edges:
                return list(new_path)
            visited.add(oid)
            q.append((oid, new_path))
    return None


def _find_next_vss(
    net,
    start_edge_id: str,
    approach_to_id: dict,
    visited_iids: set,
    max_hops: int,
    heading_tolerance_deg: float = 60.0,
):
    """BFS forward from start_edge_id, looking for the next VSS approach edge
    that lies in the SAME direction of travel as start_edge.

    The heading check is what prevents nonsensical "next VSS" matches: e.g.,
    market.SB.through goes SOUTH, so any candidate "next VSS" must also be
    reached heading roughly south. 1st (NE of market) fails this check, so
    SB-through correctly terminates with the exit tail instead of inventing
    a winding path that ends at 1st.

    Returns (next_iid, next_direction, edge_path) or None.
        edge_path = start_edge as the first element, candidate approach edge
        as the last.

    Direction-similarity logic:
        start_h    = edge_heading_out_of_node(start_edge)
                     (direction of travel as cars leave the current VSS)
        cand_h     = edge_heading_into_node(candidate)
                     (direction of travel as cars arrive at the next VSS)
        if |signed_turn(start_h, cand_h)| > tolerance → reject candidate, keep BFS
    """
    start_edge = net.getEdge(start_edge_id)
    start_h = edge_heading_out_of_node(start_edge)

    q: deque = deque()
    q.append((start_edge_id, (start_edge_id,)))
    visited_edges = {start_edge_id}

    while q:
        eid, path = q.popleft()
        if len(path) > 1 and eid in approach_to_id:
            iid, d = approach_to_id[eid]
            if iid not in visited_iids:
                cand_h = edge_heading_into_node(net.getEdge(eid))
                if abs(signed_turn(start_h, cand_h)) <= heading_tolerance_deg:
                    return iid, d, list(path)
                # else: candidate is in a wrong direction; keep BFS

        if len(path) > max_hops:
            continue

        edge = net.getEdge(eid)
        edge_from = edge.getFromNode().getID()
        for out_edge, conns in edge.getOutgoing().items():
            oid = out_edge.getID()
            if oid.startswith(":") or not conns or oid in visited_edges:
                continue
            if not out_edge.allows("passenger"):
                continue
            if out_edge.getToNode().getID() == edge_from:
                continue
            visited_edges.add(oid)
            q.append((oid, path + (oid,)))

    return None


def enumerate_patterns(
    net,
    mapped_results,
    original_data,
    internal_dirs,
    entry_hops: int = 3,
    exit_hops: int = 3,
    max_corridor_hops: int = 12,
):
    """Enumerate all deterministic flow patterns from VSS data.

    Each pattern is a complete path consisting of:
      [entry buffer]  (entry_hops edges upstream of the first VSS approach)
      + [approach edge, movement edge] for each VSS in the chain
      + [exit tail]   (exit_hops edges downstream of the last decision)

    The entry buffer and exit tail are short by design: cars should spawn
    just before the VSS region of interest and disappear just after leaving
    it. We do NOT route to the full SUMO network boundary — that would
    over-simulate areas we have no VSS data for.

    Within the VSS region, the path is built deterministically from the turn
    ratios: only "through" recurses into the next VSS intersection along the
    corridor (within max_corridor_hops); left/right always exit the corridor
    immediately into the exit tail. Each leaf has a fixed vehs/h rate
    computed from the joint VSS ratio along the path.

    Returns: (patterns: list[Pattern], unrouted: list[dict])
    """
    mapped_by_id = {r["intersection_id"]: r for r in mapped_results if r["status"] == "matched"}
    counts_by_id = {i["intersection_id"]: i.get("counts", {}) for i in original_data.get("intersections", [])}

    # approach_to_id[edge_id] = (iid, direction)
    approach_to_id: dict[str, tuple[str, str]] = {}
    for iid, mapped in mapped_by_id.items():
        for d, info in mapped["approaches"].items():
            approach_to_id[info["edge_id"]] = (iid, d)

    patterns: list[Pattern] = []
    unrouted: list[dict] = []  # patterns we couldn't assemble into a valid route

    def _emit_leaf(edges_so_far, decisions, cars, tail_start_edge):
        """Compose a complete pattern route: entry buffer + VSS path + exit tail.

        Entry: `entry_hops` edges upstream of the first VSS approach
               (= edges_so_far[0]), via straightest-incoming heuristic.
        Exit:  `exit_hops` edges downstream of `tail_start_edge`,
               via straightest-outgoing heuristic.

        The spawned vehicle drives through the entry buffer, into the VSS
        approach, follows the deterministic VSS decisions, then drives the
        exit tail and exits the simulation at the tail's last edge.
        """
        approach = edges_so_far[0]
        entry_chain = _walk_backward_straightest(net, approach, entry_hops)
        # entry_chain ends with `approach`. We prepend everything before it.
        entry_prefix = entry_chain[:-1]

        exit_tail = _walk_forward_straightest(net, tail_start_edge, exit_hops)

        full_edges = entry_prefix + list(edges_so_far) + exit_tail
        if len(full_edges) < 2:
            unrouted.append({
                "decisions": list(decisions),
                "reason": "route_too_short_after_buffer_assembly",
            })
            return

        # Defensive dedup of accidental consecutive duplicates
        cleaned = [full_edges[0]]
        for e in full_edges[1:]:
            if e != cleaned[-1]:
                cleaned.append(e)

        pattern_id = " > ".join(f"{i}.{d}.{t}" for i, d, t in decisions)
        patterns.append(Pattern(
            pattern_id=pattern_id,
            edges=cleaned,
            vehs_per_hour=cars,
            decisions=list(decisions),
        ))

    def _recurse(edges_so_far, decisions, cars, visited_iids):
        """Continue exploring from the last edge in edges_so_far.

        The car just made a turn; edges_so_far[-1] is the outgoing edge they
        rode after the turn. Try to find the next VSS intersection forward.
        """
        if cars <= 0:
            return

        current_edge = edges_so_far[-1]

        result = _find_next_vss(net, current_edge, approach_to_id, visited_iids, max_corridor_hops)

        if result is None:
            # Pattern terminates: exit corridor with a short tail
            _emit_leaf(edges_so_far, decisions, cars, current_edge)
            return

        next_iid, next_d, path_to_next = result
        # path_to_next[0] == current_edge, path_to_next[-1] == next intersection's approach edge

        next_counts = counts_by_id.get(next_iid, {}).get(next_d, {})
        next_movements = mapped_by_id[next_iid]["approaches"][next_d]["movements"]

        if not next_counts or not next_movements:
            # We hit a VSS but it has no turn data for this direction → terminate
            extended = list(edges_so_far) + list(path_to_next[1:])
            _emit_leaf(extended, decisions, cars, extended[-1])
            return

        # Conservation: split `cars` over AVAILABLE turn movements only.
        # If a VSS-reported turn has no movement edge in the SUMO net (e.g.
        # 1st.EB.right is not allowed in OSM at this junction), redistribute
        # its share proportionally among the remaining turns. Cars must go
        # somewhere; we trust VSS volumes more than VSS direction breakdown
        # when the network contradicts the data.
        available = [
            (turn, float(next_counts[turn]), next_movements[turn])
            for turn in TURNS
            if turn in next_counts and turn in next_movements and float(next_counts[turn]) > 0
        ]
        total_avail = sum(v for _, v, _ in available)
        if total_avail <= 0:
            # No usable turn movements at this junction → terminate
            extended = list(edges_so_far) + list(path_to_next[1:])
            _emit_leaf(extended, decisions, cars, extended[-1])
            return

        for turn, sub_v, move_edge in available:
            sub_cars = cars * (sub_v / total_avail)
            new_edges = list(edges_so_far) + list(path_to_next[1:]) + [move_edge]
            new_decisions = decisions + [(next_iid, next_d, turn)]
            new_visited = visited_iids | {next_iid}
            # ANY turn (left/through/right) may continue toward a next VSS,
            # subject to the direction-similarity filter inside _find_next_vss.
            # A side-street car turning left/right onto the corridor naturally
            # merges into corridor traffic and may reach the next VSS — we
            # want to model that, not arbitrarily cut it off at exit_hops.
            # The filter prevents nonsensical "wind south then back north"
            # matches (the bug that motivated the through-only rule earlier).
            _recurse(new_edges, new_decisions, sub_cars, new_visited)

    # Boundary-walk: every (mapped intersection, non-internal direction) is a tree root.
    # Same redistribution semantics as downstream junctions: the total inflow at this
    # boundary direction must be preserved; if some turn movements are unavailable in
    # the SUMO net, redistribute proportionally among the available turns.
    for iid, mapped in mapped_by_id.items():
        counts = counts_by_id.get(iid, {})
        internal_at_iid = internal_dirs.get(iid, set())
        approach_dirs = set(mapped["approaches"].keys())

        for d in approach_dirs:
            if d in internal_at_iid:
                continue
            if d not in counts:
                continue
            turn_counts = counts[d]
            movements = mapped["approaches"][d]["movements"]
            approach_edge = mapped["approaches"][d]["edge_id"]

            total_vss = sum(float(v) for v in turn_counts.values())
            available = [
                (turn, float(turn_counts[turn]), movements[turn])
                for turn in TURNS
                if turn in turn_counts and turn in movements and float(turn_counts[turn]) > 0
            ]
            total_avail = sum(v for _, v, _ in available)
            if total_avail <= 0:
                continue  # no usable turns at this boundary direction

            for turn, sub_v, move_edge in available:
                cars = total_vss * (sub_v / total_avail)
                edges_so_far = [approach_edge, move_edge]
                decisions = [(iid, d, turn)]
                visited_iids = {iid}
                # Boundary entry: any turn may continue toward a downstream
                # VSS subject to the direction-similarity filter.
                _recurse(edges_so_far, decisions, cars, visited_iids)

    return patterns, unrouted


# ---------------------------------------------------------------------------
# Route writing — emit SUMO .rou.xml with one <route>+<flow> per pattern
# ---------------------------------------------------------------------------

def compute_depart_lane(net, edges: list) -> int:
    """Choose a starting lane on edges[0] that is actually usable.

    SUMO's `departLane="best"` looks ahead and sometimes picks a lane that
    matches a downstream turn requirement but DOES NOT HAVE an outgoing
    connection to the very next edge — common with OSM-imported nets that
    contain "dead lanes" (right-turn pockets or sidewalks marked as vehicle
    lanes but with no connection). A car placed on such a lane is stuck.

    This helper walks 1-2 edges of the route and picks the FIRST lane on
    edges[0] that:
      1. allows passenger vehicles
      2. has a SUMO connection to edges[1]
      3. (preferred) whose downstream lane on edges[1] also connects to
         edges[2], maximizing the chance the car can keep going without
         awkward forced lane-changes
    """
    if not edges:
        return 0
    e0 = net.getEdge(edges[0])

    def _first_passenger_lane(edge):
        for lane in edge.getLanes():
            if "passenger" in lane.getPermissions():
                return lane.getIndex()
        return 0

    if len(edges) < 2:
        return _first_passenger_lane(e0)

    e1 = net.getEdge(edges[1])
    candidates: list = []  # (lane_idx_on_e0, lane_idx_on_e1)
    for lane in e0.getLanes():
        if "passenger" not in lane.getPermissions():
            continue
        for c in lane.getOutgoing():
            if c.getToLane().getEdge() == e1:
                candidates.append((lane.getIndex(), c.getToLane().getIndex()))
                break  # one connection per lane is enough
    if not candidates:
        return _first_passenger_lane(e0)

    # Prefer a candidate whose lane on e1 also has a connection to e2
    if len(edges) >= 3 and len(candidates) > 1:
        e2 = net.getEdge(edges[2])
        for from_idx, to_idx in candidates:
            to_lane = e1.getLane(to_idx)
            for c in to_lane.getOutgoing():
                if c.getToLane().getEdge() == e2:
                    return from_idx
    return candidates[0][0]


def write_deterministic_routes(
    net,
    patterns: Iterable[Pattern],
    route_out: Path,
    sim_end: int,
    min_vehs_per_hour: float = 0.5,
):
    """Write SUMO routes file with one <route>+<flow> per pattern.

    Patterns whose rate falls below `min_vehs_per_hour` are dropped — these
    are rounding noise (e.g., 0.3 veh/h) that produce 0–1 SUMO vehicles in
    short sims and add XML clutter without affecting behavior.
    """
    root = ET.Element("routes")
    root.set("xmlns:xsi", "http://www.w3.org/2001/XMLSchema-instance")
    root.set("xsi:noNamespaceSchemaLocation", "http://sumo.dlr.de/xsd/routes_file.xsd")
    ET.SubElement(root, "vType", {
        "id": "car_normal", "vClass": "passenger", "accel": "2.6", "decel": "4.5",
        "maxSpeed": "33.3", "sigma": "0.5", "tau": "1.2", "length": "4.5", "minGap": "2.5",
    })

    emitted = 0
    dropped_small = 0
    dropped_short = 0
    for idx, pat in enumerate(patterns):
        if pat.vehs_per_hour < min_vehs_per_hour:
            dropped_small += 1
            continue
        if len(pat.edges) < 2:
            dropped_short += 1
            continue

        route_id = f"r_{idx:04d}"
        flow_id = f"f_{idx:04d}"
        depart_lane = compute_depart_lane(net, pat.edges)
        ET.SubElement(root, "route", {
            "id": route_id,
            "edges": " ".join(pat.edges),
        })
        ET.SubElement(root, "flow", {
            "id": flow_id,
            "type": "car_normal",
            "route": route_id,
            "begin": "0",
            "end": str(sim_end),
            "vehsPerHour": f"{pat.vehs_per_hour:.2f}",
            # Manually computed: a lane on edges[0] that has a real connection
            # forward to edges[1] (avoids SUMO's "best" picking dead lanes that
            # exist in OSM but have no outgoing).
            "departLane": str(depart_lane),
            "departSpeed": "max",
        })
        emitted += 1

    route_out.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(route_out, encoding="utf-8", xml_declaration=True)
    return {
        "emitted": emitted,
        "dropped_below_min_rate": dropped_small,
        "dropped_too_short": dropped_short,
    }


# ---------------------------------------------------------------------------
# Conservation check
# ---------------------------------------------------------------------------

def check_conservation(patterns: Iterable[Pattern], original, mapped_results, internal_dirs):
    """Compare per-boundary (intersection, direction) pattern totals against
    VSS-reported inflows.

    Two distinct loss categories:
      - 'redistribution': should be 0 if redistribution is correct (we
        renormalize when downstream turn movements are missing). Non-zero
        here indicates a real bug.
      - 'no_approach': VSS reports cars on a direction whose SUMO approach
        edge doesn't exist (network/data mismatch). These cars are unrouted
        by design; the report flags them for inspection.
    """
    mapped_by_id = {r["intersection_id"]: r for r in mapped_results if r["status"] == "matched"}

    by_boundary: dict[tuple[str, str], float] = defaultdict(float)
    for p in patterns:
        if not p.decisions:
            continue
        first_iid, first_d, _ = p.decisions[0]
        by_boundary[(first_iid, first_d)] += p.vehs_per_hour

    routable_rows = []
    no_approach_rows = []

    for inter in original.get("intersections", []):
        iid = inter["intersection_id"]
        internal_at_iid = internal_dirs.get(iid, set())
        mapped = mapped_by_id.get(iid)
        approach_dirs = set(mapped["approaches"].keys()) if mapped else set()

        for d, turn_counts in inter.get("counts", {}).items():
            if d in internal_at_iid:
                continue
            exp = sum(float(v) for v in turn_counts.values())

            if mapped is None or d not in approach_dirs:
                no_approach_rows.append({
                    "intersection": iid,
                    "direction": d,
                    "lost_vehs_per_hour": exp,
                    "reason": "no_matching_approach_edge_in_sumo_net",
                })
                continue

            got = by_boundary.get((iid, d), 0.0)
            routable_rows.append({
                "intersection": iid,
                "direction": d,
                "expected_vehs_per_hour": exp,
                "got_vehs_per_hour": round(got, 4),
                "diff": round(got - exp, 4),
            })

    max_abs_diff = max((abs(r["diff"]) for r in routable_rows), default=0.0)
    total_no_approach = sum(r["lost_vehs_per_hour"] for r in no_approach_rows)
    return {
        "redistribution_ok": max_abs_diff < 1e-6,
        "max_abs_redistribution_diff": max_abs_diff,
        "routable_rows": routable_rows,
        "no_approach_loss_vehs_per_hour": total_no_approach,
        "no_approach_rows": no_approach_rows,
    }


# ---------------------------------------------------------------------------
# Arrival verification — every car must end at a network boundary edge
# ---------------------------------------------------------------------------

def verify_route_completion(tripinfo_path: Path, route_file: Path, sample_size: int = 20):
    """Verify every completed vehicle arrived at the LAST EDGE OF ITS
    ASSIGNED ROUTE. Catches any "vanished mid-route" event.

    A car that completes its full pattern arrives at the exit tail's last
    edge (its designated exit point). Anything else means the car was
    teleported/vaporized somewhere unexpected — a routing or simulation gap
    we want to surface in the report.

    Returns dict:
        total_finished              : tripinfo entry count
        arrived_at_expected_end     : count matching the route's last edge
        arrived_elsewhere           : count NOT matching
        all_complete                : bool — success criterion
        wrong_examples              : first N mismatches for debugging
    """
    tp = Path(tripinfo_path)
    rp = Path(route_file)
    if not tp.exists():
        return {"error": f"tripinfo_file_not_found: {tp}"}
    if not rp.exists():
        return {"error": f"route_file_not_found: {rp}"}

    # Build flow_id → last_edge_of_route mapping
    route_to_last_edge: dict = {}
    flow_to_route: dict = {}
    routes_root = ET.parse(rp).getroot()
    for r in routes_root.iter("route"):
        edges = (r.get("edges") or "").split()
        if edges:
            route_to_last_edge[r.get("id")] = edges[-1]
    for f in routes_root.iter("flow"):
        flow_to_route[f.get("id")] = f.get("route")

    flow_to_last_edge = {
        fid: route_to_last_edge.get(rid)
        for fid, rid in flow_to_route.items()
        if route_to_last_edge.get(rid)
    }

    total = 0
    correct = 0
    wrong: list = []
    for t in ET.parse(tp).iter("tripinfo"):
        total += 1
        vid = t.get("id", "")
        flow_id = vid.rsplit(".", 1)[0]
        expected_last = flow_to_last_edge.get(flow_id)
        if expected_last is None:
            wrong.append({"id": vid, "reason": "flow_or_route_not_found"})
            continue
        arrival_lane = t.get("arrivalLane", "")
        actual_last = arrival_lane.rsplit("_", 1)[0]
        if actual_last == expected_last:
            correct += 1
        else:
            wrong.append({
                "id": vid,
                "flow": flow_id,
                "expected_last_edge": expected_last,
                "actual_last_edge": actual_last,
                "arrival_pos": t.get("arrivalPos"),
                "duration": t.get("duration"),
            })

    return {
        "total_finished": total,
        "arrived_at_expected_end": correct,
        "arrived_elsewhere": len(wrong),
        "all_complete": len(wrong) == 0,
        "wrong_examples": wrong[:sample_size],
    }


# Back-compat alias (older external callers might still use this name)
verify_arrivals_at_boundaries = verify_route_completion


# ---------------------------------------------------------------------------
# SUMO sanity run
# ---------------------------------------------------------------------------

def run_sumo(
    net_file: Path,
    tls_file: Path,
    route_file: Path,
    sim_end: int,
    tripinfo_out: Path | None = None,
    statistic_out: Path | None = None,
):
    cmd = [
        "sumo",
        "-n", str(net_file),
        "-a", str(tls_file),
        "-r", str(route_file),
        "--begin", "0",
        "--end", str(sim_end),
        "--step-length", "1",
        "--no-step-log",
        "--ignore-route-errors",   # skip vehicles whose route hits a lane-level dead-end
        "--duration-log.statistics", "true",
    ]
    if tripinfo_out is not None:
        cmd += ["--tripinfo-output", str(tripinfo_out)]
    if statistic_out is not None:
        cmd += ["--statistic-output", str(statistic_out)]
    try:
        return subprocess.run(cmd, check=False, capture_output=True, text=True)
    except FileNotFoundError:
        return None


def _proc_summary(proc):
    if proc is None:
        return {"return_code": None, "stdout_tail": [], "stderr_tail": ["binary_not_found"]}
    return {
        "return_code": proc.returncode,
        "stdout_tail": proc.stdout.splitlines()[-30:],
        "stderr_tail": proc.stderr.splitlines()[-30:],
    }


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    ap.add_argument("--input-json", required=True)
    ap.add_argument("--net-file", required=True)
    ap.add_argument("--tls-file", required=True)
    ap.add_argument("--boundary-json-out", required=True)
    ap.add_argument("--route-out", required=True)
    ap.add_argument("--report-out", required=True)
    ap.add_argument("--patterns-out", default=None,
                    help="Optional: dump enumerated patterns to JSON for debugging")
    ap.add_argument("--tripinfo-out", default=None,
                    help="Optional: SUMO tripinfo output (for completion stats)")
    ap.add_argument("--statistic-out", default=None,
                    help="Optional: SUMO statistic output (aggregate stats)")
    ap.add_argument("--sim-end", type=int, default=1800)
    ap.add_argument("--max-match-distance-m", type=float, default=80.0)
    ap.add_argument("--entry-hops", type=int, default=0,
                    help="Edges to PREPEND upstream of each VSS approach as the entry buffer. "
                         "Default 0 (spawn directly on the approach edge — usually long enough "
                         "to absorb spawn pressure). Positive values can REDUCE throughput if "
                         "the upstream edges are short or have dead lanes; tested on SJ downtown "
                         "where entry_hops=3 dropped 1st.NB completion from 99% to 26%.")
    ap.add_argument("--exit-hops", type=int, default=3,
                    help="Edges to APPEND downstream of the last VSS decision as the exit tail. "
                         "Cars finish the route and exit the simulation at the tail's last edge. "
                         "Default 3.")
    ap.add_argument("--max-corridor-hops", type=int, default=6,
                    help="Max forward hops when searching for the next VSS intersection "
                         "along the corridor. Adjacent VSS on the same street are typically "
                         "3-5 hops apart; the heading-similarity filter (±60°) further "
                         "rejects winding non-corridor matches. Default 6.")
    ap.add_argument("--min-vehs-per-hour", type=float, default=0.5,
                    help="Drop pattern flows whose rate is below this (rounding noise).")
    ap.add_argument("--skip-sumo", action="store_true",
                    help="Skip the SUMO sanity run (just emit routes).")
    args = ap.parse_args()

    original = json.loads(Path(args.input_json).read_text(encoding="utf-8"))
    net = sumolib.net.readNet(args.net_file, withInternal=True)

    mapped = map_intersections(net, original, max_dist_m=args.max_match_distance_m)
    internal_dirs = identify_internal_inflows(net, mapped)
    boundary_json = build_boundary_json(net, original, mapped, internal_dirs)
    Path(args.boundary_json_out).write_text(json.dumps(boundary_json, indent=2), encoding="utf-8")

    # find_boundary_edges still produced (informational), but no longer used
    # for routing — entry/exit are short buffers around the VSS region.
    boundary_edges_set = find_boundary_edges(net)

    patterns, unrouted = enumerate_patterns(
        net, mapped, original, internal_dirs,
        entry_hops=args.entry_hops,
        exit_hops=args.exit_hops,
        max_corridor_hops=args.max_corridor_hops,
    )

    if args.patterns_out:
        patterns_dump = [
            {
                "pattern_id": p.pattern_id,
                "vehs_per_hour": round(p.vehs_per_hour, 4),
                "edges": p.edges,
                "decisions": p.decisions,
            }
            for p in patterns
        ]
        Path(args.patterns_out).write_text(json.dumps(patterns_dump, indent=2), encoding="utf-8")

    route_stats = write_deterministic_routes(
        net, patterns, Path(args.route_out),
        sim_end=args.sim_end,
        min_vehs_per_hour=args.min_vehs_per_hour,
    )

    conservation = check_conservation(patterns, original, mapped, internal_dirs)

    sim_proc = None
    arrivals_check = None
    if not args.skip_sumo:
        sim_proc = run_sumo(
            Path(args.net_file), Path(args.tls_file), Path(args.route_out),
            sim_end=args.sim_end,
            tripinfo_out=Path(args.tripinfo_out) if args.tripinfo_out else None,
            statistic_out=Path(args.statistic_out) if args.statistic_out else None,
        )
        if args.tripinfo_out:
            arrivals_check = verify_route_completion(
                Path(args.tripinfo_out), Path(args.route_out)
            )

    report = {
        "input_json": args.input_json,
        "approach": "deterministic_pattern_enumeration",
        "mapped_intersections": [
            {
                "id": m["intersection_id"],
                "status": m["status"],
                "distance_m": m.get("distance_to_node_m"),
                "approaches": sorted(m["approaches"].keys()),
                "reason": m.get("reason"),
            }
            for m in mapped
        ],
        "internal_directions": {iid: sorted(dirs) for iid, dirs in internal_dirs.items() if dirs},
        "network_boundary_edge_count": len(boundary_edges_set),
        "patterns_enumerated": len(patterns),
        "patterns_unrouted_to_boundary": unrouted,
        "route_emit_stats": route_stats,
        "conservation": conservation,
        "arrivals_check": arrivals_check,
        "sumo": _proc_summary(sim_proc),
    }
    Path(args.report_out).write_text(json.dumps(report, indent=2), encoding="utf-8")

    print(json.dumps({
        "boundary_json_out": args.boundary_json_out,
        "route_out": args.route_out,
        "patterns_enumerated": len(patterns),
        "patterns_emitted": route_stats["emitted"],
        "patterns_unrouted": len(unrouted),
        "redistribution_ok": conservation["redistribution_ok"],
        "no_approach_loss_vehs_per_hour": conservation["no_approach_loss_vehs_per_hour"],
        "all_complete": (arrivals_check or {}).get("all_complete"),
        "arrived_elsewhere": (arrivals_check or {}).get("arrived_elsewhere"),
        "sumo_return_code": report["sumo"]["return_code"],
    }, indent=2))


if __name__ == "__main__":
    main()

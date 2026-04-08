#!/usr/bin/env python3
"""
Safe merge: ADD-ONLY new edges/junctions from downtown into full network.

Only adds elements that exist in downtown but NOT in full network.
Does NOT replace any existing elements — avoids version conflicts.
"""

import gzip
import xml.etree.ElementTree as ET
import time

DOWNTOWN = "san_jose_downtown_gtc/osm.net.xml"
FULL     = "san_jose_full_new/osm.net.xml.gz"
OUTPUT   = "san_jose_full_new/osm_merged.net.xml.gz"


def merge_addonly(downtown_path, full_path, output_path):
    t0 = time.time()

    # --- Parse downtown ---
    print("  Parsing downtown network...")
    dt_tree = ET.parse(downtown_path)
    dt_root = dt_tree.getroot()

    dt_edges = {}       # id -> Element
    dt_junctions = {}   # id -> Element
    dt_connections = []  # list of Elements
    dt_types = {}       # id -> Element

    for elem in dt_root:
        if elem.tag == 'edge':
            dt_edges[elem.get('id')] = elem
        elif elem.tag == 'junction':
            dt_junctions[elem.get('id')] = elem
        elif elem.tag == 'connection':
            dt_connections.append(elem)
        elif elem.tag == 'type':
            dt_types[elem.get('id')] = elem

    # --- Parse full network ---
    print("  Parsing full network...")
    with gzip.open(full_path, 'rt') as f:
        full_tree = ET.parse(f)
    full_root = full_tree.getroot()

    # Collect existing IDs from full network
    full_edge_ids = set()
    full_junction_ids = set()
    full_type_ids = set()
    full_connections = set()  # (from, to, fromLane, toLane)

    for elem in full_root:
        if elem.tag == 'edge':
            full_edge_ids.add(elem.get('id'))
        elif elem.tag == 'junction':
            full_junction_ids.add(elem.get('id'))
        elif elem.tag == 'type':
            full_type_ids.add(elem.get('id'))
        elif elem.tag == 'connection':
            key = (elem.get('from'), elem.get('to'),
                   elem.get('fromLane'), elem.get('toLane'))
            full_connections.add(key)

    # --- Find NEW elements (only in downtown) ---
    new_edges = {eid: e for eid, e in dt_edges.items()
                 if eid not in full_edge_ids and not eid.startswith(':')}
    new_junctions = {jid: j for jid, j in dt_junctions.items()
                     if jid not in full_junction_ids}

    # New internal edges: only add if parent junction is NEW
    new_internal_edges = {}
    for eid, e in dt_edges.items():
        if not eid.startswith(':'):
            continue
        if eid in full_edge_ids:
            continue
        # Check if parent junction is new
        junc_id = eid.split('_')[0][1:]  # rough parse
        # More robust: check all new junction IDs
        for njid in new_junctions:
            if eid.startswith(f':{njid}_'):
                new_internal_edges[eid] = e
                break

    # New connections: only add connections between edges that exist
    # (either already in full or newly added)
    all_edge_ids = full_edge_ids | set(new_edges.keys()) | set(new_internal_edges.keys())
    new_conns = []
    for conn in dt_connections:
        from_e = conn.get('from')
        to_e = conn.get('to')
        key = (from_e, to_e, conn.get('fromLane'), conn.get('toLane'))
        if key in full_connections:
            continue  # already exists
        # Only add if both edges will exist in merged network
        if from_e in all_edge_ids and to_e in all_edge_ids:
            new_conns.append(conn)

    # New types
    new_types = {tid: t for tid, t in dt_types.items() if tid not in full_type_ids}

    print(f"\n  New elements to add:")
    print(f"    Regular edges:  {len(new_edges)}")
    print(f"    Internal edges: {len(new_internal_edges)}")
    print(f"    Junctions:      {len(new_junctions)}")
    print(f"    Connections:    {len(new_conns)}")
    print(f"    Types:          {len(new_types)}")

    # --- Insert new elements into full network ---
    # Find insertion points (add after last element of each type)
    last_type_idx = -1
    last_edge_idx = -1
    last_junction_idx = -1
    last_connection_idx = -1

    for i, elem in enumerate(full_root):
        if elem.tag == 'type':
            last_type_idx = i
        elif elem.tag == 'edge':
            last_edge_idx = i
        elif elem.tag == 'junction':
            last_junction_idx = i
        elif elem.tag == 'connection':
            last_connection_idx = i

    # Insert in reverse order of position to keep indices valid
    insertions = []

    if new_types:
        pos = last_type_idx + 1 if last_type_idx >= 0 else 0
        for t in new_types.values():
            insertions.append((pos, t))

    if new_edges or new_internal_edges:
        pos = last_edge_idx + 1
        for e in new_edges.values():
            insertions.append((pos, e))
        for e in new_internal_edges.values():
            insertions.append((pos, e))

    if new_junctions:
        pos = last_junction_idx + 1
        for j in new_junctions.values():
            insertions.append((pos, j))

    if new_conns:
        pos = last_connection_idx + 1 if last_connection_idx >= 0 else len(full_root)
        for c in new_conns:
            insertions.append((pos, c))

    # Sort by position descending so inserts don't shift later indices
    insertions.sort(key=lambda x: x[0], reverse=True)
    for pos, elem in insertions:
        full_root.insert(pos, elem)

    total_added = len(new_edges) + len(new_internal_edges) + len(new_junctions) + len(new_conns) + len(new_types)
    print(f"\n  Total elements added: {total_added}")
    print(f"  Original elements unchanged: {len(full_root) - total_added}")

    # --- Write output ---
    print(f"  Writing: {output_path}")
    ET.indent(full_tree, space='    ')
    with gzip.open(output_path, 'wt', encoding='utf-8') as f:
        full_tree.write(f, encoding='unicode', xml_declaration=True)

    dt = time.time() - t0
    print(f"  Done in {dt:.1f}s")


if __name__ == '__main__':
    merge_addonly(DOWNTOWN, FULL, OUTPUT)

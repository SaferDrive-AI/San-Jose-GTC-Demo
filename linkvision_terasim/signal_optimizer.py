from __future__ import annotations
"""Signal-plan selection rules for blocked-lane demo scenarios.

Given obstacle context (lane or direction), this module picks the most
specific available TLS program and records the selection reason.
"""

from dataclasses import dataclass
from typing import Iterable


DEFAULT_TLS_ID = "cluster_1984576776_3478559735_3478559736_3537422682_#1more"
WB_OPTIMAL_PROGRAM_ID = "1418903639#0_2"


@dataclass(frozen=True)
class SignalPlanDecision:
    tls_id: str
    program_id: str
    reason: str


def _normalize_direction(direction: str | None) -> str | None:
    if direction is None:
        return None
    normalized = direction.strip().upper()
    aliases = {
        "W": "WB",
        "WEST": "WB",
        "WESTBOUND": "WB",
        "E": "EB",
        "EAST": "EB",
        "EASTBOUND": "EB",
        "N": "NB",
        "NORTH": "NB",
        "NORTHBOUND": "NB",
        "S": "SB",
        "SOUTH": "SB",
        "SOUTHBOUND": "SB",
    }
    return aliases.get(normalized, normalized)


def _is_available(program_id: str, available_programs: set[str] | None) -> bool:
    return available_programs is None or program_id in available_programs


def choose_signal_plan(
    obstacle_lane_id: str | None = None,
    direction: str | None = None,
    available_programs: Iterable[str] | None = None,
    tls_id: str = DEFAULT_TLS_ID,
    default_program: str = "org",
    fallback_program: str = "opt",
) -> SignalPlanDecision:
    """Return the fixed signal plan for an obstacle.

    The rule mirrors the existing demo behavior: exact lane-specific programs
    win first, edge-level programs are next, then known directional optimal
    plans, then the generic optimized plan.
    """

    available = None if available_programs is None else set(available_programs)

    if obstacle_lane_id and _is_available(obstacle_lane_id, available):
        return SignalPlanDecision(tls_id, obstacle_lane_id, "exact_lane_program")

    if obstacle_lane_id:
        edge_id = obstacle_lane_id.rsplit("_", 1)[0]
        if available is not None:
            for program_id in sorted(available):
                if program_id.startswith(edge_id + "_"):
                    return SignalPlanDecision(tls_id, program_id, "edge_program")

    if _normalize_direction(direction) == "WB" and _is_available(WB_OPTIMAL_PROGRAM_ID, available):
        return SignalPlanDecision(tls_id, WB_OPTIMAL_PROGRAM_ID, "directional_optimal_plan")

    if _is_available(fallback_program, available):
        return SignalPlanDecision(tls_id, fallback_program, "fallback_opt")

    return SignalPlanDecision(tls_id, default_program, "fallback_default")

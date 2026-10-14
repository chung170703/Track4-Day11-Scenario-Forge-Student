"""Part A: read a rollout. TODO 1, 2 and 9 live here; the helpers below them are given."""
from __future__ import annotations

import numpy as np

from avsim import core

G = 9.81
HALF_EGO = core.CAR[0] / 2   # 2.3 m: half the ego's length


def ego_events(R: core.Rollout, k: int) -> list[dict]:
    """Failure events involving the ego in scenario k, earliest first (given).

    event["kind"] is "contact" (the boxes touched) or "block" (the ego's body stopped the pedestrian P1 on the
    crosswalk: pedestrians never walk into a car, so this is how a planner that stops too late fails them)."""
    return [e for e in R.events[k] if e["a"] == "ego" or e["b"] == "ego"]


def min_ttc(R: core.Rollout, cap: float = 10.0) -> np.ndarray:
    """TODO 1. (K,) smallest time-to-collision between the ego and whatever it is following, capped at `cap` s.

    - The ego's leader at frame f is R.EGO[k, f, 3] (an actor index, -1 = nothing ahead).
    - Gap is bumper to bumper along the ego's heading: lon - 2.3 - (leader length / 2), where
      lon = (x_leader - x_ego) * hx_ego + (z_leader - z_ego) * hz_ego (the leader's position seen from the ego) and
      2.3 is half the ego's length. Leader length: core.LEN[j] for actor j; use 0.3 m as the half-length of a
      pedestrian. Clip the gap at 0.
    - TTC = gap / (v_ego - v_leader), only while the ego is closing in by more than 0.1 m/s.
    - A scenario where the ego crashed (see ego_events) returns 0.
    Shapes: R.F is (K, NF, N, 7) with x, z, hx, hz, v, a, d; core.IX["ego"] is the ego's actor index.
    Vectorise over frames (a Python loop over 481 frames x K scenarios is too slow for Part C).
    """
    K, NF = R.F.shape[0], R.F.shape[1]
    ego_idx = core.IX["ego"]
    ego_F = R.F[:, :, ego_idx, :]  # (K, NF, 7)
    
    lead_idx = R.EGO[:, :, 3].astype(int)  # (K, NF)
    has_leader = lead_idx >= 0
    
    clamped_lead = np.clip(lead_idx, 0, None)
    k_grid = np.arange(K)[:, None]
    f_grid = np.arange(NF)[None, :]
    lead_F = R.F[k_grid, f_grid, clamped_lead]  # (K, NF, 7)
    
    lead_half_len = np.where(has_leader, core.LEN[clamped_lead] / 2.0, 0.0)
    
    rx = lead_F[..., 0] - ego_F[..., 0]
    rz = lead_F[..., 1] - ego_F[..., 1]
    lon = rx * ego_F[..., 2] + rz * ego_F[..., 3]
    
    gap = np.maximum(lon - 2.3 - lead_half_len, 0.0)
    dv = ego_F[..., 4] - lead_F[..., 4]
    
    valid = has_leader & (dv > 0.1)
    ttc = np.where(valid, gap / np.maximum(dv, 1e-6), np.inf)
    
    min_ttc_k = np.min(ttc, axis=1)  # (K,)
    res = np.minimum(cap, min_ttc_k)
    
    for k in range(K):
        if ego_events(R, k):
            res[k] = 0.0
            
    return res


def contact_type(R: core.Rollout, k: int, event: dict) -> str:
    """TODO 2. 'front' (the ego ran into it), 'rear' (it ran into the ego) or 'side', at the contact frame.

    Put the other actor in the ego's frame: lon = r . h, lat = -r_x h_z + r_z h_x with h = (hx, hz) of the ego.
    'front' if lon > 2.3 and |lat| < 1.6; 'rear' if lon < -2.3 and |lat| < 1.6; otherwise 'side'.
    """
    other_name = event["b"] if event["a"] == "ego" else event["a"]
    other_idx = core.IX[other_name]
    f = event["f"]
    
    ego_state = R.F[k, f, core.IX["ego"]]
    other_state = R.F[k, f, other_idx]
    
    rx = other_state[0] - ego_state[0]
    rz = other_state[1] - ego_state[1]
    hx = ego_state[2]
    hz = ego_state[3]
    
    lon = rx * hx + rz * hz
    lat = -rx * hz + rz * hx
    
    if lon > 2.3 and abs(lat) < 1.6:
        return "front"
    elif lon < -2.3 and abs(lat) < 1.6:
        return "rear"
    else:
        return "side"


def progress_metrics(R: core.Rollout) -> tuple[np.ndarray, np.ndarray]:
    """TODO 9. Two (K,) arrays: the distance the ego covers in the 24 s (sum of frame-to-frame steps, metres),
    and the number of frames where the ego brakes harder than 3 m/s^2 (acceleration column < -3)."""
    ego_idx = core.IX["ego"]
    ego_x = R.F[:, :, ego_idx, 0]
    ego_z = R.F[:, :, ego_idx, 1]
    ego_a = R.F[:, :, ego_idx, 5]
    
    dx = np.diff(ego_x, axis=1)
    dz = np.diff(ego_z, axis=1)
    steps = np.hypot(dx, dz)
    dist = np.sum(steps, axis=1)
    
    harsh = np.sum(ego_a < -3.0, axis=1)
    return dist, harsh


# ---------------------------------------------------------------- given helpers

def pair_min_ttc(R, follower, leader, cap=10.0, lat_max=2.2, leader_vmin=-1.0):
    """(K,) smallest TTC of `follower` running into `leader` while they share a corridor."""
    F = R.F
    a, b = F[:, :, core.IX[follower]], F[:, :, core.IX[leader]]
    rx, rz = b[..., 0] - a[..., 0], b[..., 1] - a[..., 1]
    lon = rx * a[..., 2] + rz * a[..., 3]
    lat = -rx * a[..., 3] + rz * a[..., 2]
    gap = np.maximum(lon - core.LEN[core.IX[follower]] / 2 - core.LEN[core.IX[leader]] / 2, 0.0)
    dv = a[..., 4] - b[..., 4]
    ok = (lon > 0) & (np.abs(lat) < lat_max) & (dv > 0.1) & (b[..., 4] >= leader_vmin)
    ttc = np.where(ok, gap / np.maximum(dv, 1e-6), np.inf)
    return np.minimum(cap, ttc.min(1))


def ped_room(R, cap=30.0):
    """(K,) smallest stopping room [m] left while the crossing pedestrian is in the ego's lane
    (distance to them minus the ego's braking distance at mu*g). Negative = could not have stopped."""
    e, p = R.F[:, :, core.IX["ego"]], R.F[:, :, core.IX["P1"]]
    mu = np.asarray(R.params["mu"])[:, None]
    inlane = np.abs(p[..., 1] - e[..., 1]) < 1.9
    ahead = p[..., 0] - e[..., 0] - HALF_EGO
    room = np.where(inlane & (ahead > -1.0), ahead - e[..., 4] ** 2 / (2 * mu * G), cap)
    return np.clip(room.min(1), -cap, cap)

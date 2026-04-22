"""Pressure-test the range-weighted angular-diversity metric D_w against canonical
trajectory geometries. The goal is to check that thresholds rank the cases
single-pass < parallel-passes < crossing-passes < loop, and to pick D_min.

Metric:
    D_w(V) = sum_{i<j} w_i w_j acos(d_i . d_j) / sum_{i<j} w_i w_j
    w_i = exp(-rho_i / rho_0)
where d_i is the unit vector from the voxel to the sensor at return i, and
rho_i is the range from sensor to voxel.
"""

import numpy as np

RHO_0 = 10.0
RHO_MAX = 20.0
N_MIN = 10
VOXEL = np.array([0.0, 0.0, 1.0])  # voxel at ~breast height, 1 m above ground
SENSOR_HEIGHT = 1.5  # Fairy on a pack/tripod
RNG = np.random.default_rng(0)


def observe(sensor_positions: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Given sensor positions (N,3), return unit vectors voxel->sensor and ranges."""
    vecs = sensor_positions - VOXEL
    ranges = np.linalg.norm(vecs, axis=1)
    dirs = vecs / ranges[:, None]
    return dirs, ranges


def diversity(dirs: np.ndarray, ranges: np.ndarray, rho_0: float = RHO_0) -> float:
    """Weighted mean pairwise angle in radians."""
    n = len(dirs)
    if n < 2:
        return 0.0
    w = np.exp(-ranges / rho_0)
    # Pairwise dot products, clipped for acos stability
    dots = np.clip(dirs @ dirs.T, -1.0, 1.0)
    angles = np.arccos(dots)
    # Upper triangle only (i<j)
    iu = np.triu_indices(n, k=1)
    a = angles[iu]
    ww = (w[:, None] * w[None, :])[iu]
    return float(np.sum(ww * a) / np.sum(ww))


def sample_along_line(start: np.ndarray, end: np.ndarray, n: int) -> np.ndarray:
    t = np.linspace(0, 1, n)[:, None]
    return start * (1 - t) + end * t


def case_single_pass(n: int = 40, offset: float = 3.0) -> np.ndarray:
    """Straight line 3 m to the side of the voxel."""
    start = np.array([-8.0, offset, SENSOR_HEIGHT])
    end = np.array([8.0, offset, SENSOR_HEIGHT])
    return sample_along_line(start, end, n)


def case_parallel_passes(n: int = 40, offset: float = 3.0) -> np.ndarray:
    """Two passes on the *same* side — same incidence hemisphere."""
    a = sample_along_line(
        np.array([-8.0, offset, SENSOR_HEIGHT]),
        np.array([8.0, offset, SENSOR_HEIGHT]),
        n // 2,
    )
    b = sample_along_line(
        np.array([-8.0, offset + 1.5, SENSOR_HEIGHT]),
        np.array([8.0, offset + 1.5, SENSOR_HEIGHT]),
        n // 2,
    )
    return np.vstack([a, b])


def case_crossing_passes(n: int = 40, offset: float = 3.0) -> np.ndarray:
    """Two orthogonal passes crossing near the voxel."""
    a = sample_along_line(
        np.array([-8.0, offset, SENSOR_HEIGHT]),
        np.array([8.0, offset, SENSOR_HEIGHT]),
        n // 2,
    )
    b = sample_along_line(
        np.array([offset, -8.0, SENSOR_HEIGHT]),
        np.array([offset, 8.0, SENSOR_HEIGHT]),
        n // 2,
    )
    return np.vstack([a, b])


def case_opposite_passes(n: int = 40, offset: float = 3.0) -> np.ndarray:
    """Two parallel passes on opposite sides of the voxel."""
    a = sample_along_line(
        np.array([-8.0, offset, SENSOR_HEIGHT]),
        np.array([8.0, offset, SENSOR_HEIGHT]),
        n // 2,
    )
    b = sample_along_line(
        np.array([-8.0, -offset, SENSOR_HEIGHT]),
        np.array([8.0, -offset, SENSOR_HEIGHT]),
        n // 2,
    )
    return np.vstack([a, b])


def case_loop(n: int = 40, radius: float = 3.0) -> np.ndarray:
    """Full loop around the voxel."""
    theta = np.linspace(0, 2 * np.pi, n, endpoint=False)
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)
    z = np.full_like(x, SENSOR_HEIGHT)
    return np.stack([x, y, z], axis=1)


def case_half_loop(n: int = 40, radius: float = 3.0) -> np.ndarray:
    """Semicircle — sampled from one side of S^2 projection."""
    theta = np.linspace(0, np.pi, n)
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)
    z = np.full_like(x, SENSOR_HEIGHT)
    return np.stack([x, y, z], axis=1)


def evaluate(name: str, positions: np.ndarray) -> None:
    dirs, ranges = observe(positions)
    # Keep only returns within RHO_MAX — glancing far hits are gated out.
    mask = ranges <= RHO_MAX
    dirs, ranges = dirs[mask], ranges[mask]
    n = len(dirs)
    d_w = diversity(dirs, ranges)
    mean_range = float(np.mean(ranges)) if n else float("nan")
    gate_n = n >= N_MIN
    print(
        f"{name:22s}  N={n:3d}  D_w={d_w:.3f} rad ({np.degrees(d_w):5.1f}°)"
        f"  mean_rho={mean_range:4.1f} m  gate_N={gate_n}"
    )


def main() -> None:
    print(f"rho_0={RHO_0}  rho_max={RHO_MAX}  N_min={N_MIN}")
    print("-" * 72)
    cases = [
        ("single pass",        case_single_pass()),
        ("parallel passes",    case_parallel_passes()),
        ("opposite passes",    case_opposite_passes()),
        ("crossing passes",    case_crossing_passes()),
        ("half loop",          case_half_loop()),
        ("full loop",          case_loop()),
    ]
    for name, pos in cases:
        evaluate(name, pos)

    # Sensitivity: how does D_w scale with offset distance (range weighting bite)?
    print()
    print("Crossing passes — sensitivity to offset (range-weighting bite):")
    for offset in [1.0, 2.0, 3.0, 5.0, 8.0, 12.0]:
        pos = case_crossing_passes(n=40, offset=offset)
        dirs, ranges = observe(pos)
        mask = ranges <= RHO_MAX
        d_w = diversity(dirs[mask], ranges[mask])
        print(
            f"  offset={offset:4.1f} m  N={mask.sum():3d}  "
            f"D_w={d_w:.3f} rad ({np.degrees(d_w):5.1f}°)"
        )


if __name__ == "__main__":
    main()

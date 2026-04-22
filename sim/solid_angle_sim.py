"""Solid-angle coverage metric: bin viewpoint directions onto a tessellation of
S^2 and count occupied cells. Also compute azimuth-only coverage for stems.

Tessellations compared:
  - icosa20:  20 face centers of an icosahedron (uniform-ish on S^2)
  - healpix12: HEALPix nside=1 (12 equal-area pixels)
  - healpix48: HEALPix nside=2 (48 equal-area pixels)
  - azimuth16: 16 azimuth bins ignoring elevation (stem metric)
"""

import numpy as np

RHO_0 = 10.0
RHO_MAX = 20.0
N_MIN = 10
W_MIN = 0.1  # per-cell weight floor below which we treat the cell as unoccupied
VOXEL = np.array([0.0, 0.0, 1.0])
SENSOR_HEIGHT = 1.5


# ---------------- tessellations ----------------

def icosahedron_centers() -> np.ndarray:
    """20 face-center unit vectors of a regular icosahedron."""
    phi = (1 + 5 ** 0.5) / 2
    verts = np.array([
        [-1,  phi, 0], [ 1,  phi, 0], [-1, -phi, 0], [ 1, -phi, 0],
        [0, -1,  phi], [0,  1,  phi], [0, -1, -phi], [0,  1, -phi],
        [ phi, 0, -1], [ phi, 0,  1], [-phi, 0, -1], [-phi, 0,  1],
    ], dtype=float)
    verts /= np.linalg.norm(verts, axis=1, keepdims=True)
    faces = [
        (0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11),
        (1, 5, 9), (5, 11, 4), (11, 10, 2), (10, 7, 6), (7, 1, 8),
        (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9),
        (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1),
    ]
    centers = np.array([verts[a] + verts[b] + verts[c] for a, b, c in faces])
    centers /= np.linalg.norm(centers, axis=1, keepdims=True)
    return centers


def healpix_centers(nside: int) -> np.ndarray:
    """HEALPix pixel centers as unit vectors, RING scheme. Pure-numpy impl."""
    npix = 12 * nside * nside
    ipix = np.arange(npix)
    ncap = 2 * nside * (nside - 1)

    z = np.empty(npix)
    phi = np.empty(npix)

    # North polar cap
    cap = ipix < ncap
    ip = ipix[cap] + 1
    iring = ((1 + np.sqrt(1 + 2 * ip)) / 2).astype(int)
    iphi = ip - 2 * iring * (iring - 1)
    z[cap] = 1.0 - (iring * iring) / (3.0 * nside * nside)
    phi[cap] = (iphi - 0.5) * np.pi / (2.0 * iring)

    # Equatorial belt
    belt = (ipix >= ncap) & (ipix < npix - ncap)
    ip = ipix[belt] - ncap
    iring_b = ip // (4 * nside) + nside
    iphi_b = ip % (4 * nside) + 1
    fodd = 0.5 * (1 + ((iring_b + nside) & 1))
    z[belt] = (2 * nside - iring_b) * 2.0 / (3.0 * nside)
    phi[belt] = (iphi_b - fodd) * np.pi / (2.0 * nside)

    # South polar cap
    sp = ipix >= npix - ncap
    ip = npix - ipix[sp]
    iring_s = ((1 + np.sqrt(2 * ip - 1)) / 2).astype(int)
    iphi_s = 4 * iring_s + 1 - (ip - 2 * iring_s * (iring_s - 1))
    z[sp] = -1.0 + (iring_s * iring_s) / (3.0 * nside * nside)
    phi[sp] = (iphi_s - 0.5) * np.pi / (2.0 * iring_s)

    sin_t = np.sqrt(np.maximum(0.0, 1.0 - z * z))
    return np.stack([sin_t * np.cos(phi), sin_t * np.sin(phi), z], axis=1)


# ---------------- observation model ----------------

def observe(positions: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    vecs = positions - VOXEL
    ranges = np.linalg.norm(vecs, axis=1)
    dirs = vecs / ranges[:, None]
    return dirs, ranges


def coverage_s2(dirs: np.ndarray, weights: np.ndarray, centers: np.ndarray) -> int:
    """Assign each direction to nearest cell; return number of cells whose
    accumulated weight clears W_MIN."""
    if len(dirs) == 0:
        return 0
    cell = np.argmax(dirs @ centers.T, axis=1)
    acc = np.bincount(cell, weights=weights, minlength=len(centers))
    return int(np.sum(acc >= W_MIN))


def coverage_azimuth(dirs: np.ndarray, weights: np.ndarray, nbins: int) -> int:
    if len(dirs) == 0:
        return 0
    az = np.arctan2(dirs[:, 1], dirs[:, 0])  # [-pi, pi]
    bins = ((az + np.pi) / (2 * np.pi) * nbins).astype(int) % nbins
    acc = np.bincount(bins, weights=weights, minlength=nbins)
    return int(np.sum(acc >= W_MIN))


# ---------------- trajectory cases ----------------

def sample_line(start, end, n):
    t = np.linspace(0, 1, n)[:, None]
    return np.asarray(start) * (1 - t) + np.asarray(end) * t


def case_single(n=40, off=3.0):
    return sample_line([-8, off, SENSOR_HEIGHT], [8, off, SENSOR_HEIGHT], n)


def case_parallel(n=40, off=3.0):
    a = sample_line([-8, off,       SENSOR_HEIGHT], [8, off,       SENSOR_HEIGHT], n // 2)
    b = sample_line([-8, off + 1.5, SENSOR_HEIGHT], [8, off + 1.5, SENSOR_HEIGHT], n // 2)
    return np.vstack([a, b])


def case_opposite(n=40, off=3.0):
    a = sample_line([-8,  off, SENSOR_HEIGHT], [8,  off, SENSOR_HEIGHT], n // 2)
    b = sample_line([-8, -off, SENSOR_HEIGHT], [8, -off, SENSOR_HEIGHT], n // 2)
    return np.vstack([a, b])


def case_crossing(n=40, off=3.0):
    a = sample_line([-8, off, SENSOR_HEIGHT], [8, off, SENSOR_HEIGHT], n // 2)
    b = sample_line([off, -8, SENSOR_HEIGHT], [off, 8, SENSOR_HEIGHT], n // 2)
    return np.vstack([a, b])


def case_half_loop(n=40, r=3.0):
    th = np.linspace(0, np.pi, n)
    return np.stack([r * np.cos(th), r * np.sin(th), np.full_like(th, SENSOR_HEIGHT)], axis=1)


def case_full_loop(n=40, r=3.0):
    th = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return np.stack([r * np.cos(th), r * np.sin(th), np.full_like(th, SENSOR_HEIGHT)], axis=1)


# ---------------- evaluation ----------------

def evaluate(cases, tess_centers, tess_names, az_bins=16):
    hdr = f"{'case':20s}  {'N':>3s}  " + "  ".join(f"{t:>8s}" for t in tess_names) + f"  {'az'+str(az_bins):>6s}"
    print(hdr)
    print("-" * len(hdr))
    for name, positions in cases:
        dirs, ranges = observe(positions)
        mask = ranges <= RHO_MAX
        dirs, ranges = dirs[mask], ranges[mask]
        w = np.exp(-ranges / RHO_0)
        cells = [coverage_s2(dirs, w, c) for c in tess_centers]
        az = coverage_azimuth(dirs, w, az_bins)
        cell_str = "  ".join(f"{c:>3d}/{len(tc):<3d}" for c, tc in zip(cells, tess_centers))
        print(f"{name:20s}  {len(dirs):>3d}  {cell_str}  {az:>3d}/{az_bins:<3d}")


def main():
    print(f"rho_0={RHO_0}  rho_max={RHO_MAX}  W_min={W_MIN}")
    tess = [
        ("icosa20",   icosahedron_centers()),
        ("hp12",      healpix_centers(1)),
        ("hp48",      healpix_centers(2)),
    ]
    tess_names   = [t[0] for t in tess]
    tess_centers = [t[1] for t in tess]

    cases = [
        ("single pass",     case_single()),
        ("parallel passes", case_parallel()),
        ("opposite passes", case_opposite()),
        ("crossing passes", case_crossing()),
        ("half loop",       case_half_loop()),
        ("full loop",       case_full_loop()),
    ]

    print("\n== Baseline (offset=3 m) ==")
    evaluate(cases, tess_centers, tess_names)

    print("\n== Dense sampling (n=200, offset=3 m) — mimics real LiDAR rate ==")
    dense = [
        ("single pass",     case_single(n=200)),
        ("parallel passes", case_parallel(n=200)),
        ("opposite passes", case_opposite(n=200)),
        ("crossing passes", case_crossing(n=200)),
        ("half loop",       case_half_loop(n=200)),
        ("full loop",       case_full_loop(n=200)),
    ]
    evaluate(dense, tess_centers, tess_names)

    print("\n== Farther offset (5 m) — checks range-weighting bite ==")
    far = [
        ("single pass",     case_single(off=5.0)),
        ("parallel passes", case_parallel(off=5.0)),
        ("opposite passes", case_opposite(off=5.0)),
        ("crossing passes", case_crossing(off=5.0)),
        ("full loop",       case_full_loop(r=5.0)),
    ]
    evaluate(far, tess_centers, tess_names)

    # Separation table: what k_min cleanly splits good from bad on icosa20?
    print("\n== Ranking (icosa20, n=40, offset=3 m) ==")
    for name, pos in cases:
        dirs, ranges = observe(pos)
        mask = ranges <= RHO_MAX
        w = np.exp(-ranges[mask] / RHO_0)
        k = coverage_s2(dirs[mask], w, tess_centers[0])
        print(f"  {name:20s}  k={k:2d}/20")


if __name__ == "__main__":
    main()

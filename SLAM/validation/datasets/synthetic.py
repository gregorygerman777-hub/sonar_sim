"""Render a synthetic pool-like monocular sequence with exact ground truth.

Scene (world z up, metres): a tiled floor (0.25 m tiles, dark grout), a band of pebbles, and a
rough rock mound in the middle, all as a height field with an albedo texture. The camera flies an
arc around the rock looking at it, like the arc in Dr. Negahdaripour's pool sequence. The
repetitive tile and pebble texture is the point: it reproduces the failure mode seen on the pool data
while the true trajectory is known exactly.

Rendering is ray marching against the height field (coarse steps, then bisection), Lambertian shading
from a fixed light, exponential attenuation towards a water colour, and Gaussian pixel noise.

Writes data_external/synthetic/<name>/{images/000000.png.., K.txt, groundtruth_tum.txt, scene.json}.
"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.ndimage import gaussian_filter, map_coordinates
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]

EXTENT = 3.0          # scene covers [-EXTENT, EXTENT]^2 metres
HEIGHT_RES = 0.004    # metres per height sample
TEX_RES = 0.002       # metres per texture sample


def value_noise(shape, scale_px, rng, octaves=4):
    out = np.zeros(shape)
    amp, total = 1.0, 0.0
    for o in range(octaves):
        s = max(scale_px / 2 ** o, 1.0)
        small = rng.random((int(shape[0] / s) + 2, int(shape[1] / s) + 2))
        big = cv2.resize(small, (shape[1], shape[0]), interpolation=cv2.INTER_CUBIC)
        out += amp * big
        total += amp
        amp *= 0.5
    return out / total


def build_scene(seed=7):
    rng = np.random.default_rng(seed)
    n_h = int(2 * EXTENT / HEIGHT_RES)
    n_t = int(2 * EXTENT / TEX_RES)
    ht = np.linspace(-EXTENT, EXTENT, n_h)
    tt = np.linspace(-EXTENT, EXTENT, n_t)
    HX, HY = np.meshgrid(ht, ht)
    TX, TY = np.meshgrid(tt, tt)

    # --- height field
    r = np.hypot(HX, HY)
    rock = 0.38 * np.exp(-(r / 0.45) ** 2) + 0.12 * np.exp(-((HX - 0.25) ** 2 + (HY + 0.1) ** 2) / 0.08)
    rough = gaussian_filter(rng.normal(size=HX.shape), 6) * 0.9
    height = rock * (1 + rough * (rock > 0.01))
    pebbles = []
    for _ in range(900):
        rr = rng.uniform(0.6, 1.9)
        a = rng.uniform(0, 2 * np.pi)
        pebbles.append((rr * np.cos(a), rr * np.sin(a), rng.uniform(0.01, 0.035), rng.uniform(0.25, 0.75)))
    peb_h = np.zeros_like(HX)
    for x, y, rad, _ in pebbles:
        i0, i1 = int((y - rad + EXTENT) / HEIGHT_RES), int((y + rad + EXTENT) / HEIGHT_RES) + 1
        j0, j1 = int((x - rad + EXTENT) / HEIGHT_RES), int((x + rad + EXTENT) / HEIGHT_RES) + 1
        sub = (slice(max(i0, 0), i1), slice(max(j0, 0), j1))
        d2 = (HX[sub] - x) ** 2 + (HY[sub] - y) ** 2
        bump = 0.6 * rad * np.sqrt(np.clip(1 - d2 / rad ** 2, 0, 1))
        peb_h[sub] = np.maximum(peb_h[sub], bump)
    height = np.maximum(height, peb_h)

    # --- albedo texture (RGB, 0..1)
    tile = 0.25
    gx = np.abs(((TX + 10 * tile) % tile) - tile / 2)
    gy = np.abs(((TY + 10 * tile) % tile) - tile / 2)
    grout = (np.minimum(tile / 2 - gx, tile / 2 - gy) < 0.006)
    tile_id = (np.floor(TX / tile) * 131 + np.floor(TY / tile) * 71).astype(int)
    tile_var = (np.sin(tile_id * 12.9898) * 43758.5453) % 1.0
    albedo = np.zeros(TX.shape + (3,))
    base = np.array([0.72, 0.80, 0.84])
    albedo[:] = base * (0.92 + 0.08 * tile_var[..., None])
    fine = value_noise(TX.shape, 40, rng, 3)
    albedo *= (0.9 + 0.2 * fine[..., None])
    albedo[grout] = [0.28, 0.30, 0.32]
    tr = np.hypot(TX, TY)
    rock_tex = value_noise(TX.shape, 14, rng, 4)
    rock_col = np.stack((0.30 + 0.45 * rock_tex, 0.25 + 0.38 * rock_tex, 0.20 + 0.30 * rock_tex), -1)
    rock_mask = tr < 0.75
    blend = np.clip((0.75 - tr) / 0.15, 0, 1)[..., None]
    albedo = np.where(rock_mask[..., None], blend * rock_col + (1 - blend) * albedo, albedo)
    for x, y, rad, shade in pebbles:
        i0, i1 = int((y - rad + EXTENT) / TEX_RES), int((y + rad + EXTENT) / TEX_RES) + 1
        j0, j1 = int((x - rad + EXTENT) / TEX_RES), int((x + rad + EXTENT) / TEX_RES) + 1
        sub = (slice(max(i0, 0), i1), slice(max(j0, 0), j1))
        inside = (TX[sub] - x) ** 2 + (TY[sub] - y) ** 2 < rad ** 2
        col = np.array([shade, shade * 0.93, shade * 0.85])
        region = albedo[sub]
        region[inside] = col * (0.85 + 0.3 * fine[sub][inside][:, None])
    return height.astype(np.float32), albedo.astype(np.float32)


def sample(grid, res, x, y, order=1):
    rows = (y + EXTENT) / res
    cols = (x + EXTENT) / res
    if grid.ndim == 2:
        return map_coordinates(grid, [rows, cols], order=order, mode="nearest")
    return np.stack([map_coordinates(grid[..., c], [rows, cols], order=order, mode="nearest")
                     for c in range(grid.shape[2])], -1)


def look_at(position, target, up=(0, 0, 1.0)):
    z = target - position
    z /= np.linalg.norm(z)
    x = np.cross(z, up)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack((x, y, z))   # R_wc: columns are camera axes in world


def caustics(x, y, t):
    """Moving caustic light network on the floor (0..1), a function of world position and time in seconds."""
    wx = x + 0.06 * np.sin(7.0 * y + 1.3 * t) + 0.04 * np.sin(11.0 * x - 0.7 * t)
    wy = y + 0.06 * np.sin(6.0 * x - 1.1 * t) + 0.04 * np.sin(9.0 * y + 0.9 * t)
    out = np.zeros_like(x)
    for angle, freq, speed in ((0.3, 9.0, 1.7), (1.4, 11.0, -1.3), (2.5, 10.0, 1.1)):
        a = wx * np.cos(angle) + wy * np.sin(angle)
        out += (1.0 - np.abs(np.sin(freq * a + speed * t))) ** 8
    return np.clip(out / 1.5, 0.0, 1.0)


def render(height, albedo, K, R_wc, c, size, rng, noise=2.0, caustic_time=None, caustic_gain=0.9):
    w, h = size
    u, v = np.meshgrid(np.arange(w) + 0.5, np.arange(h) + 0.5)
    rays_c = np.stack(((u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], np.ones_like(u)), -1).reshape(-1, 3)
    d = rays_c @ R_wc.T
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    n = len(d)
    s_prev = np.zeros(n)
    s_hit = np.full(n, np.nan)
    step = 0.01
    idx = np.arange(n)
    s = 0.0
    for _ in range(1200):
        s_new = s + step
        p = c + d[idx] * s_new
        above = p[:, 2] > sample(height, HEIGHT_RES, p[:, 0], p[:, 1])
        hit = ~above
        s_hit[idx[hit]] = s_new
        s_prev[idx[hit]] = s
        keep = above & (np.abs(p[:, 0]) < EXTENT) & (np.abs(p[:, 1]) < EXTENT) & (p[:, 2] < 3.0)
        idx = idx[keep]
        s = s_new
        if len(idx) == 0:
            break
    found = np.isfinite(s_hit)
    lo, hi = s_prev[found], s_hit[found]
    df = d[found]
    for _ in range(10):
        mid = 0.5 * (lo + hi)
        p = c + df * mid[:, None]
        above = p[:, 2] > sample(height, HEIGHT_RES, p[:, 0], p[:, 1])
        lo = np.where(above, mid, lo)
        hi = np.where(above, hi, mid)
    s_f = 0.5 * (lo + hi)
    p = c + df * s_f[:, None]
    e = 0.004
    hx = (sample(height, HEIGHT_RES, p[:, 0] + e, p[:, 1]) - sample(height, HEIGHT_RES, p[:, 0] - e, p[:, 1])) / (2 * e)
    hy = (sample(height, HEIGHT_RES, p[:, 0], p[:, 1] + e) - sample(height, HEIGHT_RES, p[:, 0], p[:, 1] - e)) / (2 * e)
    normal = np.column_stack((-hx, -hy, np.ones_like(hx)))
    normal /= np.linalg.norm(normal, axis=1, keepdims=True)
    light = np.array([0.4, -0.3, 1.0])
    light /= np.linalg.norm(light)
    shade = 0.35 + 0.65 * np.clip(normal @ light, 0, 1)
    col = sample(albedo, TEX_RES, p[:, 0], p[:, 1]) * shade[:, None]
    if caustic_time is not None:
        col = col * (1.0 + caustic_gain * caustics(p[:, 0], p[:, 1], caustic_time))[:, None]
    water = np.array([0.20, 0.42, 0.45])
    att = np.exp(-0.12 * s_f)[:, None]
    col = att * col + (1 - att) * water
    image = np.tile(water, (n, 1))
    image[found] = col
    image = image.reshape(h, w, 3) * 255 + rng.normal(0, noise, (h, w, 3))
    return np.clip(image, 0, 255).astype(np.uint8)[..., ::-1]   # RGB to BGR for OpenCV


def trajectory(n_frames, radius=1.9, altitude=1.35, arc_deg=300.0):
    poses = []
    for i in range(n_frames):
        a = np.radians(-60 + arc_deg * i / (n_frames - 1))
        pos = np.array([radius * np.cos(a), radius * np.sin(a), altitude + 0.12 * np.sin(3 * a)])
        target = np.array([0.15 * np.sin(2 * a), 0.1 * np.cos(a), 0.12])
        poses.append((look_at(pos, target), pos))
    return poses


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--name", default="synthetic_pool")
    parser.add_argument("--frames", type=int, default=240)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--focal", type=float, default=500.0)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--caustics", action="store_true",
                        help="add moving caustic light patterns on the floor (non-rigid, like the real pool)")
    parser.add_argument("--out", type=Path, default=ROOT / "data_external/synthetic")
    args = parser.parse_args()
    out = args.out / args.name
    (out / "images").mkdir(parents=True, exist_ok=True)
    K = np.array([[args.focal, 0, args.width / 2], [0, args.focal, args.height / 2], [0, 0, 1.0]])
    np.savetxt(out / "K.txt", K)
    height, albedo = build_scene(args.seed)
    rng = np.random.default_rng(args.seed + 1)
    poses = trajectory(args.frames)
    with open(out / "groundtruth_tum.txt", "w") as fh:
        fh.write("# timestamp tx ty tz qx qy qz qw (camera to world, OpenCV camera axes, world z up, metres)\n")
        for i, (R, c) in enumerate(poses):
            q = Rotation.from_matrix(R).as_quat()
            fh.write(f"{i / 10:.6f} {c[0]:.9f} {c[1]:.9f} {c[2]:.9f} {q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}\n")
    for i, (R, c) in enumerate(poses):
        image = render(height, albedo, K, R, c, (args.width, args.height), rng,
                       caustic_time=(i / 10.0) if args.caustics else None)
        cv2.imwrite(str(out / "images" / f"{i:06d}.png"), image)
        if i % 20 == 0:
            print(f"rendered {i}/{args.frames}", flush=True)
    (out / "scene.json").write_text(json.dumps(dict(seed=args.seed, frames=args.frames, extent_m=EXTENT,
                                                     tile_m=0.25, focal_px=args.focal, caustics=bool(args.caustics),
                                                     size=[args.width, args.height]), indent=2))
    np.save(out / "height.npy", height[::5, ::5])   # 2 cm grid, for the ground truth scene figure


if __name__ == "__main__":
    main()

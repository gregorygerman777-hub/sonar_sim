"""Stage A: extract and cache ORB features for every frame once."""
import pickle
import sys
import time
from pathlib import Path

import cv2
import numpy as np

REPO = Path("/Users/gregsobe/sonar_sim")
sys.path.insert(0, str(REPO / "SLAM/optical_benchmark_20260918"))
import stereo_vo  # noqa: E402

DATA = Path("/Users/gregsobe/Downloads/Archive")
OUT = Path("/Users/gregsobe/sonar_sim/tmp/pool_vo_20260919/phd")
N_FRAMES = 117
ASSUMED_K = np.array([[900.0, 0.0, 512.0], [0.0, 900.0, 384.0], [0.0, 0.0, 1.0]])


def load_gray(i):
    im = cv2.imread(str(DATA / f"opt{i}.bmp"), cv2.IMREAD_COLOR)
    if im is None:
        raise FileNotFoundError(DATA / f"opt{i}.bmp")
    return cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)


def main():
    front = stereo_vo.StereoFrontEnd(ASSUMED_K, baseline_m=1.0)
    frames = []
    started = time.perf_counter()
    for i in range(1, N_FRAMES + 1):
        gray = load_gray(i)
        feat = front.extract(gray)
        # image-quality stats for later failure diagnosis
        lap_var = float(cv2.Laplacian(gray, cv2.CV_64F).var())
        fft = np.fft.fftshift(np.fft.fft2(gray.astype(float)))
        mag = np.abs(fft)
        h, w = gray.shape
        cy, cx = h // 2, w // 2
        r = min(h, w) // 8
        yy, xx = np.ogrid[:h, :w]
        mask_low = (yy - cy) ** 2 + (xx - cx) ** 2 <= r ** 2
        high_freq_energy = float(mag[~mask_low].sum() / mag.sum())
        frames.append(dict(index=i, keypoints=feat.keypoints, descriptors=feat.descriptors,
                            n_features=int(len(feat.keypoints)), lap_var=lap_var,
                            high_freq_energy=high_freq_energy, contrast=float(gray.std()),
                            mean_intensity=float(gray.mean())))
        if i % 20 == 0:
            print(f"  extracted {i}/{N_FRAMES}  ({time.perf_counter() - started:.1f}s elapsed)")
    with open(OUT / "features.pkl", "wb") as fh:
        pickle.dump(dict(frames=frames, K=ASSUMED_K), fh)
    print(f"done in {time.perf_counter() - started:.1f}s -> {OUT / 'features.pkl'}")


if __name__ == "__main__":
    main()

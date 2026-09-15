# External sonar SLAM research evaluation

[Results and interpretation](REPORT.md) · [Frozen protocol](PROTOCOL.md)

This folder evaluates `python/slam.py` and the C++ `PlanarSlam` backend with real
DFKI ARIS sonar sequences. It includes odometry, loop-closure ablation, temporal
sampling sensitivity, and separate development and previously untested recordings.
The synthetic interactive SLAM laboratory is still launched with `launch_slam.command`.

## Reproduce

Use an isolated Python environment; install the repository extension plus
`requirements-public-benchmark.txt`. Run from the repository root:

```sh
python SLAM/research/test_evaluation.py
python tests/test_public_uxo.py
python tests/test_slam.py
python SLAM/research/fetch_holdout.py
python SLAM/research/evaluate.py --data data_external/dfki_uxo_2024/uxo_samples --output SLAM/research/runs/new-development --split development
python SLAM/research/evaluate.py --data data_external/dfki_uxo_2024/holdout/recordings --output SLAM/research/runs/new-heldout --split heldout
```

The existing development sample download/extraction and the beam table are in
`data_external/dfki_uxo_2024`. The sample archive's published MD5 was verified in the
initial evaluation. `fetch_holdout.py` downloads only the first 1.884 GB solid block
plus the archive index, extracts the three prespecified sequences, and checks each
published member CRC. It does not claim to verify the entire 16.658 GB archive.
All complete sequences are processed. Feature caches are keyed by script, calibration,
whitelisted metadata, and raw-image hashes. ATE uses proper SE(2), never fitted scale.

## Reading the experiment

- `evaluate.py`: all-frame extraction, ICP odometry, at most 81 graph keyframes,
  legacy loop proposals, reference-only scoring, and raw trajectory/constraint export.
- `audit.py`: independent homogeneous transform composition and timing/range audit.
- `test_evaluation.py`: exact accelerated-loader equivalence and known-motion checks.
- `summarize.py`: paired sequence summaries, descriptive sequence bootstrap, figures.
- `runs/`: actual development/heldout measurements and source snapshots.
- `calibration/`: published beam centers and mounting transforms, retained for audit.

The graph uses accumulated pair variances and keyframes to keep the dense C++ solver
tractable. It is a documented adaptation, not an exact reproduction of the 37-ping
synthetic experiment. Before/after-loop comparisons share timestamps within each run;
across strides keyframe timestamps may differ. RPE uses nearest 1-second pairs within
0.15 seconds; unavailable pairs are reported as missing, not as zero error.

Sources: [DFKI data release](https://zenodo.org/records/13778485),
[publisher tools](https://github.com/dfki-ric/uxo-dataset2024), publisher commit
`323ac6cf45bb6129806c8e3ed8ec31e22e1a392d`.
This is a same-site object-centric tank test, not independent-site navigation validation.

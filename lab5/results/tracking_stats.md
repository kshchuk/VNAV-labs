# Lab 5 — Feature Tracking Statistics

Generated: 2026-08-06

## Two-frame mode (`box.png` / `box_in_scene.png`)

| Descriptor | Keypoints img1 | Keypoints img2 | Matches | Good matches | Inliers | Inlier ratio | Samples |
|------------|---------------:|---------------:|--------:|-------------:|--------:|-------------:|--------:|
| SIFT       | 604 | 969 | 604 | 102 | 82 | 80.4% | 1 |

Images: `box.png`, `box_in_scene.png`  
Launch: `two_frames_tracking.launch.yaml descriptor:=SIFT show_images:=true save_images:=true`

> SIFT was run three times; results were identical each run.

---

## Video mode (rosbag)

**Bag:** `30fps_424x240_2018-10-01-18-35-06` (424×240 @ 30 fps)  
**Launch:** `video_tracking.launch.yaml descriptor:=<NAME> bag_name:=30fps_424x240_2018-10-01-18-35-06`  
**Note:** Runs stopped manually with Ctrl+C; final stats printed on shutdown (destructor).

| Descriptor | Avg keypoints img1 | Avg keypoints img2 | Avg matches | Avg good matches | Avg inliers | Avg inlier ratio | Frame pairs |
|------------|-------------------:|-------------------:|------------:|-----------------:|------------:|-----------------:|------------:|
| SIFT       | 384.2 | 384.1 | 384.2 | 228.6 | 218.3 | 94.7% | 1051 |
| AKAZE      | 148.7 | 148.7 | 148.7 | 126.9 | 125.4 | 98.8% | 1310 |
| ORB        | 320.6 | 320.6 | 320.6 | 234.0 | 229.9 | 97.9% | 1538 |
| BRISK      | 360.4 | 360.3 | 360.4 | 235.7 | 230.2 | 97.4% | 1513 |
| Harris+LK  | 58.5 | 58.0 | 58.0 | N/A | 57.9 | 99.9% | 1571 |

> **Harris+LK** (Deliverable 7): `goodFeaturesToTrack` (Harris) + `calcOpticalFlowPyrLK`. No descriptor matching step — "good matches" is N/A; `status` from LK indicates tracked points.

### Rankings (video, same bag)

| Metric | Best |
|--------|------|
| Most keypoints per frame | **SIFT** (~384) |
| Most good matches | **BRISK** (~236) |
| Most inliers | **BRISK** (~230) |
| Highest inlier ratio | **Harris+LK** (99.9%) |

On consecutive video frames (small baseline), all descriptors perform much better than on the static `box` / `box_in_scene` pair, where viewpoint and scale change are large.

---

## Saved visualization files (two-frame, SIFT, `save_images:=true`)

Written to Docker workspace `/workspace` → host path `nav/`:

| File | Description |
|------|-------------|
| `keypoints_1.png` | Keypoints on `box.png` |
| `keypoints_2.png` | Keypoints on `box_in_scene.png` |
| `good_matches.png` | Matches after Lowe ratio test |
| `inliers_outliers.png` | Inliers (green) / outliers (red) |

---

## Raw log lines (final stats per run)

### Two-frame — SIFT
```
Avg. Keypoints 1 Size: 604
Avg. Keypoints 2 Size: 969
Avg. Number of matches: 604
Avg. Number of good matches: 102
Avg. Number of Inliers: 82
Avg. Inliers ratio: 0.803922
Num. of samples: 1
```

### Video — SIFT (1051 samples)
```
Avg. Keypoints 1 Size: 384.153
Avg. Keypoints 2 Size: 384.061
Avg. Number of matches: 384.153
Avg. Number of good matches: 228.607
Avg. Number of Inliers: 218.318
Avg. Inliers ratio: 0.946741
Num. of samples: 1051
```

### Video — AKAZE (1310 samples)
```
Avg. Keypoints 1 Size: 148.702
Avg. Keypoints 2 Size: 148.655
Avg. Number of matches: 148.702
Avg. Number of good matches: 126.903
Avg. Number of Inliers: 125.38
Avg. Inliers ratio: 0.987728
Num. of samples: 1310
```

### Video — ORB (1538 samples)
```
Avg. Keypoints 1 Size: 320.648
Avg. Keypoints 2 Size: 320.604
Avg. Number of matches: 320.648
Avg. Number of good matches: 233.997
Avg. Number of Inliers: 229.88
Avg. Inliers ratio: 0.979173
Num. of samples: 1538
```

### Video — BRISK (1513 samples)
```
Avg. Keypoints 1 Size: 360.386
Avg. Keypoints 2 Size: 360.324
Avg. Number of matches: 360.386
Avg. Number of good matches: 235.703
Avg. Number of Inliers: 230.237
Avg. Inliers ratio: 0.973729
Num. of samples: 1513
```

### Video — Harris+LK (1571 samples)
```
Avg. Keypoints 1 Size: 58.5334
Avg. Keypoints 2 Size: 58.0216
Avg. Number of matches: 58.0216
Avg. Number of good matches: NA
Avg. Number of Inliers: 57.9427
Avg. Inliers ratio: 0.998691
Num. of samples: 1571
```

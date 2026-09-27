#!/usr/bin/env python3
"""Графіки RPE для Lab 6: python3 plot_rpe.py [results_dir]"""
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

RESULTS = Path(sys.argv[1] if len(sys.argv) > 1 else "VNAV-labs/lab6/results")
METHODS_2D = {"5pt": "5-point (Nistér)", "8pt": "8-point (Longuet-Higgins)",
              "2pt": "2-point (known R)", "5pt_noransac": "5-point, no RANSAC"}


def load(name):
    path = RESULTS / f"rpe_{name}.csv"
    if not path.exists():
        return None
    df = pd.read_csv(path)
    df["time"] = df["stamp"] - df["stamp"].iloc[0]
    return df


def plot(methods, column, ylabel, title, out, logy=False):
    fig, ax = plt.subplots(figsize=(10, 4))
    for key, label in methods.items():
        df = load(key)
        if df is not None:
            ax.plot(df["time"], df[column], lw=0.9, label=label)
    ax.set(xlabel="час, с", ylabel=ylabel, title=title)
    if logy:
        ax.set_yscale("log")
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(RESULTS / out, dpi=150)


def summary(methods):
    for key, label in methods.items():
        df = load(key)
        if df is None:
            continue
        tr, rot = df.trans_err, df.rot_err_deg
        print(f"{label:28s} n={len(df):4d}")
        print(f"  trans: median={tr.median():.4f}  "
              f"<0.5: {(tr < 0.5).mean():.1%}  <0.1: {(tr < 0.1).mean():.1%}")
        print(f"  rot:   median={rot.median():.3f} deg  "
              f"<1: {(rot < 1).mean():.1%}  <0.1: {(rot < 0.1).mean():.1%}")


plot(METHODS_2D, "trans_err", "RPE трансляції (одиничні вектори)",
     "2D-2D: похибка напрямку трансляції", "rpe_trans_2d.png")
plot(METHODS_2D, "rot_err_deg", "RPE обертання, °",
     "2D-2D: похибка обертання", "rpe_rot_2d.png", logy=True)
plot({"arun": "Arun 3-point"}, "trans_err", "RPE трансляції, м",
     "3D-3D: похибка трансляції", "rpe_trans_3d.png")
plot({"arun": "Arun 3-point"}, "rot_err_deg", "RPE обертання, °",
     "3D-3D: похибка обертання", "rpe_rot_3d.png")
summary({**METHODS_2D, "arun": "Arun 3-point"})

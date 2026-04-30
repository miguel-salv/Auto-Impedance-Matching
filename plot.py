#!/usr/bin/env python3
"""
Plot a saved VSWR telemetry CSV file.

Expected format:

  host_time_s,device_millis,vswr,forward_v,reverse_v,motor1_pos_rad,motor2_pos_rad,at_match

VSWR outside [-50, 50] are dropped.

For large files, data is binned for display (mean + VSWR min/max ribbon) so plots
stay readable without manual downsampling.
"""
from __future__ import annotations

import argparse
import csv
import os
import sys
from typing import Any, Dict, Tuple

import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np

DATA_CSV_DIR = os.path.join("data", "csv")
DATA_HTML_DIR = os.path.join("data", "html")

# Drop VSWR outliers (garbage spikes)
VSWR_ABS_MAX = 50.0


def resolve_input_csv(path: str) -> str:
    """Use path if it exists; for a bare name (or ./name), try data/csv/name."""
    if os.path.isfile(path):
        return path
    name = os.path.basename(path)
    if name and (os.path.dirname(path) in ("", ".") or path == name):
        candidate = os.path.join(DATA_CSV_DIR, name)
        if os.path.isfile(candidate):
            return candidate
    return path


def resolve_html_output(html_arg: str | None, input_csv: str) -> str | None:
    """Map --html to a path; bare names and empty (auto) go under data/html/."""
    if html_arg is None:
        return None
    if html_arg == "":
        stem = os.path.splitext(os.path.basename(input_csv))[0]
        return os.path.join(DATA_HTML_DIR, f"{stem}.html")
    if os.path.isabs(html_arg) or os.path.dirname(html_arg):
        return html_arg
    return os.path.join(DATA_HTML_DIR, html_arg)


def parse_float_or_nan(s: str) -> float:
    t = s.strip().lower()
    if t in ("", "nan", "none"):
        return float("nan")
    return float(s)


def parse_args():
    parser = argparse.ArgumentParser(description="Plot a saved VSWR telemetry CSV file.")
    parser.add_argument(
        "file",
        help="CSV log file. Bare names are resolved to ./path first, then data/csv/<name>.",
    )
    parser.add_argument(
        "--max-plot-points",
        type=int,
        default=4000,
        help="Max horizontal bins for drawing (dense files are aggregated; default 4000).",
    )
    parser.add_argument(
        "--raw",
        action="store_true",
        help="Plot every sample as lines (slow/ugly on huge files). Overrides aggregation.",
    )
    parser.add_argument(
        "--minutes",
        action="store_true",
        help="Use minutes on the time axis instead of seconds.",
    )
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="Open an interactive Plotly chart (zoom/pan/hover) in the browser.",
    )
    parser.add_argument(
        "--html",
        nargs="?",
        const="",
        default=None,
        metavar="PATH",
        help="Write Plotly HTML. Omit PATH for data/html/<input_stem>.html; "
        "a bare filename is written under data/html/.",
    )
    return parser.parse_args()


def load_csv(path: str) -> Tuple[list, int, int]:
    rows = []
    skipped_format = 0
    skipped_invalid = 0

    with open(path, newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if header is None:
            return rows, 0, 0

        for raw in reader:
            if len(raw) != 8:
                skipped_format += 1
                continue
            try:
                epoch = float(raw[0])
                vswr = float(raw[2])
                motor1 = parse_float_or_nan(raw[5])
                motor2 = parse_float_or_nan(raw[6])
                match = int(float(raw[7])) != 0
            except (ValueError, IndexError):
                skipped_format += 1
                continue

            if not np.isfinite(epoch) or not np.isfinite(vswr):
                skipped_invalid += 1
                continue

            if not (-VSWR_ABS_MAX <= vswr <= VSWR_ABS_MAX):
                skipped_invalid += 1
                continue

            rows.append((epoch, vswr, motor1, motor2, match))

    return rows, skipped_format, skipped_invalid


def _nanmean(a: np.ndarray) -> float:
    m = np.nanmean(a)
    return float(m) if np.isfinite(m) else float("nan")


def bin_aggregate(
    t: np.ndarray,
    vswr: np.ndarray,
    motor1: np.ndarray,
    motor2: np.ndarray,
    matched: np.ndarray,
    max_bins: int,
) -> Tuple[np.ndarray, dict]:
    """Reduce arrays to at most max_bins contiguous slices (mean + VSWR min/max)."""
    n = len(t)
    if n <= max_bins:
        return t, {
            "mode": "raw",
            "vswr": vswr,
            "vswr_min": vswr,
            "vswr_max": vswr,
            "motor1": motor1,
            "motor2": motor2,
            "matched": matched,
        }

    edges = np.linspace(0, n, max_bins + 1, dtype=np.int64)
    centers = []
    v_mean = []
    v_min = []
    v_max = []
    m1_mean = []
    m2_mean = []
    match_last = []

    for i in range(max_bins):
        lo = int(edges[i])
        hi = int(edges[i + 1])
        if hi <= lo:
            continue
        sl = slice(lo, hi)
        centers.append(np.mean(t[sl]))
        v_mean.append(np.mean(vswr[sl]))
        v_min.append(np.min(vswr[sl]))
        v_max.append(np.max(vswr[sl]))
        m1_mean.append(_nanmean(motor1[sl]))
        m2_mean.append(_nanmean(motor2[sl]))
        match_last.append(float(matched[hi - 1]))

    return np.asarray(centers), {
        "mode": "binned",
        "vswr": np.asarray(v_mean),
        "vswr_min": np.asarray(v_min),
        "vswr_max": np.asarray(v_max),
        "motor1": np.asarray(m1_mean),
        "motor2": np.asarray(m2_mean),
        "matched": np.asarray(match_last),
    }


def plot_plotly(
    tx: np.ndarray,
    pack: Dict[str, Any],
    xlabel: str,
    title: str,
    *,
    mid_panel: str,
    show_browser: bool,
    html_path: str | None,
) -> None:
    try:
        import plotly.graph_objects as go
        from plotly.subplots import make_subplots
    except ImportError as exc:
        raise SystemExit(
            "Plotly is required for --interactive / --html. "
            "Install with:  python -m pip install plotly"
        ) from exc

    if mid_panel == "motors":
        n_rows = 3
        match_row = 3
        v_spacing = 0.06
        sub_titles = ("VSWR", "Motor positions", "Match")
        fig_height = 950
    else:
        n_rows = 2
        match_row = 2
        v_spacing = 0.1
        sub_titles = ("VSWR", "Match")
        fig_height = 720

    fig = make_subplots(
        rows=n_rows,
        cols=1,
        shared_xaxes=True,
        vertical_spacing=v_spacing,
        subplot_titles=sub_titles,
    )

    # VSWR ribbon + mean
    if pack["mode"] == "binned":
        x_band = np.concatenate([tx, tx[::-1]])
        y_band = np.concatenate([pack["vswr_max"], pack["vswr_min"][::-1]])
        fig.add_trace(
            go.Scatter(
                x=x_band,
                y=y_band,
                fill="toself",
                fillcolor="rgba(107, 174, 214, 0.35)",
                line=dict(color="rgba(255,255,255,0)", width=0),
                name="VSWR range (per bin)",
                legendgroup="vswr",
                hoverinfo="skip",
            ),
            row=1,
            col=1,
        )
        fig.add_trace(
            go.Scatter(
                x=tx,
                y=pack["vswr"],
                mode="lines",
                line=dict(color="#08519c", width=2),
                name="VSWR (mean)",
                legendgroup="vswr",
            ),
            row=1,
            col=1,
        )
    else:
        fig.add_trace(
            go.Scatter(
                x=tx,
                y=pack["vswr"],
                mode="lines",
                line=dict(color="#08519c", width=1),
                name="VSWR",
            ),
            row=1,
            col=1,
        )

    fig.add_hline(y=1.2, line_dash="dash", line_color="#238b45", row=1, col=1)
    fig.add_hline(y=1.4, line_dash="dash", line_color="#cb6816", row=1, col=1)

    if mid_panel == "motors":
        fig.add_trace(
            go.Scatter(
                x=tx,
                y=pack["motor1"],
                mode="lines",
                line=dict(color="#756bb1", width=2),
                name="Motor 1 (rad)",
            ),
            row=2,
            col=1,
        )
        fig.add_trace(
            go.Scatter(
                x=tx,
                y=pack["motor2"],
                mode="lines",
                line=dict(color="#31a354", width=2),
                name="Motor 2 (rad)",
            ),
            row=2,
            col=1,
        )

    fig.add_trace(
        go.Scatter(
            x=tx,
            y=pack["matched"],
            mode="lines",
            line=dict(shape="hv", width=0),
            fill="tozeroy",
            fillcolor="rgba(158, 202, 225, 0.45)",
            name="At match",
        ),
        row=match_row,
        col=1,
    )

    fig.update_layout(
        title=dict(text=title, font=dict(size=13)),
        height=fig_height,
        hovermode="x unified",
        legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1),
        margin=dict(t=80, b=60),
    )
    fig.update_xaxes(title_text=xlabel, row=match_row, col=1)
    fig.update_yaxes(title_text="VSWR", row=1, col=1)
    if mid_panel == "motors":
        fig.update_yaxes(title_text="rad", row=2, col=1)
    fig.update_yaxes(
        title_text="Match",
        range=[-0.12, 1.25],
        tickmode="array",
        tickvals=[0, 1],
        ticktext=["NO", "YES"],
        row=match_row,
        col=1,
    )

    config = {
        "scrollZoom": True,
        "displayModeBar": True,
        "modeBarButtonsToRemove": ["lasso2d", "select2d"],
    }

    if html_path:
        fig.write_html(html_path, config=config)
        print(f"Wrote interactive HTML: {html_path}")

    if show_browser:
        fig.show(config=config)


def apply_style():
    plt.rcParams.update(
        {
            "figure.facecolor": "#f8f9fa",
            "axes.facecolor": "#ffffff",
            "axes.edgecolor": "#cccccc",
            "axes.labelcolor": "#333333",
            "text.color": "#222222",
            "xtick.color": "#444444",
            "ytick.color": "#444444",
            "grid.color": "#dddddd",
            "grid.linestyle": "-",
            "grid.linewidth": 0.8,
            "font.size": 10,
        }
    )


def main():
    args = parse_args()

    csv_path = resolve_input_csv(args.file)
    if not os.path.isfile(csv_path):
        tried = os.path.join(DATA_CSV_DIR, os.path.basename(args.file))
        print(
            f"File not found: {args.file!r}\n"
            f"  (looked for {csv_path!r}; for bare filenames also try {tried!r})",
            file=sys.stderr,
        )
        sys.exit(1)
    if csv_path != args.file:
        print(f"Using data file: {csv_path}")

    rows, skipped_fmt, skipped_inv = load_csv(csv_path)

    if not rows:
        print("No valid rows found.", file=sys.stderr)
        sys.exit(1)

    print(
        f"Loaded {len(rows):,} valid rows  "
        f"(skipped {skipped_fmt:,} format errors, {skipped_inv:,} invalid / incomplete rows)"
    )

    epoch = np.array([r[0] for r in rows], dtype=np.float64)
    vswr = np.array([r[1] for r in rows], dtype=np.float64)
    motor1 = np.array([r[2] for r in rows], dtype=np.float64)
    motor2 = np.array([r[3] for r in rows], dtype=np.float64)
    matched = np.array([float(r[4]) for r in rows], dtype=np.float64)

    has_motors = np.isfinite(motor1).any() or np.isfinite(motor2).any()
    mid_panel = "motors" if has_motors else "none"

    t_sec = epoch - epoch[0]
    duration_min = t_sec[-1] / 60.0
    print(f"Run duration: {duration_min:.1f} minutes  ({t_sec[-1]:.0f} s)")

    if args.minutes or t_sec[-1] > 180:
        t_plot = t_sec / 60.0
        xlabel = "Time (min)"
        use_minutes = True
    else:
        t_plot = t_sec
        xlabel = "Time (s)"
        use_minutes = False

    max_bins = len(t_plot) if args.raw else max(200, args.max_plot_points)
    tx, pack = bin_aggregate(t_plot, vswr, motor1, motor2, matched, max_bins)

    if pack["mode"] == "binned":
        print(f"Display: {len(tx):,} time bins (mean + VSWR min/max band from raw samples)")

    if args.interactive or args.html is not None:
        html_out = resolve_html_output(args.html, csv_path)
        if html_out:
            parent = os.path.dirname(os.path.abspath(html_out))
            if parent:
                os.makedirs(parent, exist_ok=True)
        plot_plotly(
            tx,
            pack,
            xlabel,
            csv_path,
            mid_panel=mid_panel,
            show_browser=args.interactive,
            html_path=html_out,
        )
        return

    apply_style()

    if mid_panel == "motors":
        fig, axes = plt.subplots(
            3,
            1,
            sharex=True,
            figsize=(14, 9),
            dpi=100,
            gridspec_kw={"height_ratios": [2.2, 1.5, 1.0]},
        )
        ax_vswr, ax_motor, ax_match = axes
    else:
        fig, axes = plt.subplots(
            2,
            1,
            sharex=True,
            figsize=(14, 7),
            dpi=100,
            gridspec_kw={"height_ratios": [2.6, 1.0]},
        )
        ax_vswr, ax_match = axes

    fig.suptitle(csv_path, fontsize=9, color="#666666", y=0.995)

    # --- VSWR: ribbon + mean ---
    if pack["mode"] == "binned":
        ax_vswr.fill_between(
            tx,
            pack["vswr_min"],
            pack["vswr_max"],
            alpha=0.35,
            color="#6baed6",
            linewidth=0,
            label="VSWR range (per bin)",
        )
        ax_vswr.plot(
            tx,
            pack["vswr"],
            color="#08519c",
            linewidth=1.4,
            label="VSWR (mean)",
        )
    else:
        ax_vswr.plot(tx, pack["vswr"], color="#08519c", linewidth=0.7, alpha=0.85, label="VSWR")

    ax_vswr.axhline(1.2, color="#238b45", linestyle="--", linewidth=1.0, alpha=0.9, label="Match ≤ 1.2")
    ax_vswr.axhline(1.4, color="#cb6816", linestyle="--", linewidth=1.0, alpha=0.9, label="Unmatch ≥ 1.4")
    ax_vswr.set_ylabel("VSWR")
    ax_vswr.legend(loc="upper right", fontsize=8, framealpha=0.92)
    ax_vswr.grid(True)

    # --- Motors (only when data exists) ---
    if mid_panel == "motors":
        ax_motor.plot(tx, pack["motor1"], color="#756bb1", linewidth=1.1, label="Motor 1 (rad)")
        ax_motor.plot(tx, pack["motor2"], color="#31a354", linewidth=1.1, label="Motor 2 (rad)")
        ax_motor.set_ylabel("Motor pos (rad)")
        ax_motor.legend(loc="upper right", fontsize=8, framealpha=0.92)
        ax_motor.grid(True)

    # --- Match strip ---
    ax_match.fill_between(tx, pack["matched"], step="post", alpha=0.45, color="#9ecae1", label="At match")
    ax_match.set_ylim(-0.12, 1.25)
    ax_match.yaxis.set_major_formatter(
        ticker.FuncFormatter(lambda v, _: "YES" if v >= 0.5 else "NO")
    )
    ax_match.set_yticks([0, 1])
    ax_match.set_ylabel("Match")
    ax_match.set_xlabel(xlabel)
    ax_match.legend(loc="upper right", fontsize=8, framealpha=0.92)
    ax_match.grid(True)

    if use_minutes and not args.minutes:
        print("Note: using minutes on x-axis (run longer than 180 s). Use --minutes to force.")

    plt.tight_layout()
    plt.subplots_adjust(top=0.96)
    plt.show()


if __name__ == "__main__":
    main()

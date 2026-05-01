#!/usr/bin/env python3
"""
Plot a saved VSWR telemetry CSV file.

Expected format:

  host_time_s,device_millis,vswr,forward_v,reverse_v,motor1_pos_rad,motor2_pos_rad,at_match

VSWR outside [0, 50] are dropped.

For large files, plotting uses time-based downsampling (default every 0.5 s),
keeping real samples instead of VSWR averages.
"""
from __future__ import annotations

import argparse
import csv
import os
import sys
from datetime import datetime, timedelta
from typing import Any, Dict, Tuple

import numpy as np

DATA_CSV_DIR = os.path.join("data", "csv")
DATA_HTML_DIR = os.path.join("data", "html")
DATA_MERMAID_DIR = os.path.join("data", "mermaid")

# Drop VSWR outliers (garbage spikes)
VSWR_MAX = 4.0


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


def resolve_mermaid_output(mermaid_arg: str | None, input_csv: str) -> str | None:
    """Map --mermaid to a path; bare names and empty (auto) go under data/mermaid/."""
    if mermaid_arg is None:
        return None
    if mermaid_arg == "":
        stem = os.path.splitext(os.path.basename(input_csv))[0]
        return os.path.join(DATA_MERMAID_DIR, f"{stem}.mmd")
    if os.path.isabs(mermaid_arg) or os.path.dirname(mermaid_arg):
        return mermaid_arg
    return os.path.join(DATA_MERMAID_DIR, mermaid_arg)


def _write_mermaid_series_chart(
    path: str,
    tx: np.ndarray,
    series: np.ndarray,
    series_label: str,
    xlabel: str,
    title: str,
    y_axis_bounds: tuple[float, float] | None = None,
    render_mode: str = "line",
) -> None:
    """
    Write Mermaid xychart-beta code for one telemetry series.
    """
    if tx.size == 0:
        raise ValueError("Cannot write Mermaid chart with no samples.")

    def fmt_vals(arr: np.ndarray) -> list[str]:
        data = np.asarray(arr, dtype=np.float64).copy()
        if data.size == 0:
            return []
        finite = np.isfinite(data)
        if not finite.any():
            data[:] = 0.0
        else:
            first = int(np.argmax(finite))
            data[:first] = data[first]
            for i in range(first + 1, data.size):
                if not np.isfinite(data[i]):
                    data[i] = data[i - 1]
        return [f"{float(y):.3f}".rstrip("0").rstrip(".") for y in data]

    series_vals = fmt_vals(series)

    # Use numeric range axis to avoid huge categorical x-axis lists.
    x_start = float(tx[0])
    x_end = float(tx[-1])
    if x_end <= x_start:
        x_end = x_start + 1.0
    finite_vals = np.asarray(series, dtype=np.float64)
    finite_vals = finite_vals[np.isfinite(finite_vals)]
    if finite_vals.size:
        y_min = float(np.min(finite_vals))
        y_max = float(np.max(finite_vals))
    else:
        y_min = 0.0
        y_max = 1.0
    if y_axis_bounds is None:
        y_lo = np.floor(y_min * 10.0) / 10.0
        y_hi = np.ceil(y_max * 10.0) / 10.0
        if y_hi <= y_lo:
            y_hi = y_lo + 1.0
    else:
        y_lo, y_hi = y_axis_bounds

    plot_stmt = "line" if render_mode == "line" else "bar"

    lines = [
        "xychart-beta",
        f'    title "{os.path.basename(title)} {series_label}"',
        f'    x-axis "{xlabel}" {x_start:g} --> {x_end:g}',
        f'    y-axis "{series_label}" {y_lo:g} --> {y_hi:g}',
        f'    {plot_stmt} [{", ".join(series_vals)}]',
    ]

    parent = os.path.dirname(os.path.abspath(path))
    if parent:
        os.makedirs(parent, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def _write_mermaid_match_gantt(
    path: str,
    tx: np.ndarray,
    matched: np.ndarray,
    title: str,
    *,
    time_unit: str,
) -> None:
    """Write Mermaid gantt code for match/no-match intervals."""
    if tx.size == 0 or matched.size == 0 or tx.size != matched.size:
        raise ValueError("Cannot write Mermaid match gantt with invalid sample arrays.")

    tm = np.asarray(tx, dtype=np.float64).copy()
    if time_unit == "seconds":
        tm = tm / 60.0  # Mermaid gantt format below uses minute values.

    state = np.asarray(matched >= 0.5, dtype=bool)
    change_idx = np.flatnonzero(np.diff(state.astype(np.int8)) != 0) + 1
    starts = np.concatenate(([0], change_idx))

    pos_steps = np.diff(tm)
    pos_steps = pos_steps[np.isfinite(pos_steps) & (pos_steps > 0)]
    min_step = float(np.min(pos_steps)) if pos_steps.size else 1.0

    def to_min_token(v: float) -> int:
        return int(np.floor(float(v)))

    base_dt = datetime(2000, 1, 1, 0, 0, 0)

    def fmt_dt(minute_offset: int) -> str:
        return (base_dt + timedelta(minutes=int(minute_offset))).strftime("%Y-%m-%d %H:%M")

    lines = [
        "%%{init: {'themeVariables': {"
        "'sectionBkgColor': '#ffffff00', "
        "'sectionBkgColor2': '#ffffff00', "
        "'sectionBorderColor': '#94a3b8', "
        "'taskBkgColor': '#2563eb', "
        "'taskBorderColor': '#1d4ed8', "
        "'activeTaskBkgColor': '#16a34a', "
        "'activeTaskBorderColor': '#15803d', "
        "'doneTaskBkgColor': '#64748b', "
        "'doneTaskBorderColor': '#475569', "
        "'critTaskBkgColor': '#dc2626', "
        "'critTaskBorderColor': '#b91c1c', "
        "'ganttSectionBkgColor': '#ffffff00', "
        "'ganttSectionBkgColor2': '#ffffff00', "
        "'ganttSectionBorderColor': '#94a3b8', "
        "'ganttTaskBkgColor': '#2563eb', "
        "'ganttTaskBorderColor': '#1d4ed8', "
        "'ganttActiveTaskBkgColor': '#16a34a', "
        "'ganttActiveTaskBorderColor': '#15803d', "
        "'ganttDoneTaskBkgColor': '#64748b', "
        "'ganttDoneTaskBorderColor': '#475569', "
        "'ganttCritTaskBkgColor': '#dc2626', "
        "'ganttCritTaskBorderColor': '#b91c1c', "
        "'lineColor': '#64748b'"
        "}}}%%",
        "gantt",
        f'    title {os.path.basename(title)} Match Timeline',
        "    dateFormat  YYYY-MM-DD HH:mm",
        "    axisFormat  %H:%M",
        "",
        "    section .",
    ]

    # Build a single monotonic boundary list so adjacent tasks do not overlap.
    # Start boundary is floored, interior transition boundaries are rounded to nearest
    # minute, and final boundary is ceiled to include the tail.
    boundaries = [to_min_token(float(tm[starts[0]]))]
    if starts.size > 1:
        for idx in starts[1:]:
            boundaries.append(int(np.rint(float(tm[idx]))))
    boundaries.append(int(np.ceil(float(tm[-1]))))

    # Enforce strictly increasing boundaries to satisfy Mermaid duration parsing.
    for i in range(1, len(boundaries)):
        if boundaries[i] <= boundaries[i - 1]:
            boundaries[i] = boundaries[i - 1] + 1

    for i, run_start in enumerate(starts):
        start_tok = boundaries[i]
        end_tok = boundaries[i + 1]
        label = "YES" if state[run_start] else "NO"
        status = "active" if state[run_start] else "crit"
        lines.append(f"    {label} :{status}, {fmt_dt(start_tok)}, {fmt_dt(end_tok)}")

    parent = os.path.dirname(os.path.abspath(path))
    if parent:
        os.makedirs(parent, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def write_mermaid_xycharts(
    path: str,
    tx: np.ndarray,
    pack: Dict[str, np.ndarray],
    xlabel: str,
    title: str,
) -> list[str]:
    """Write one Mermaid file per telemetry series and return file paths."""
    abs_path = os.path.abspath(path)
    out_dir = os.path.dirname(abs_path)
    stem, ext = os.path.splitext(os.path.basename(abs_path))
    if not ext:
        ext = ".mmd"

    targets = [
        ("vswr", pack["vswr"], "VSWR", None, "line"),
        ("motor1", pack["motor1"], "Motor1", None, "line"),
        ("motor2", pack["motor2"], "Motor2", None, "line"),
    ]
    written = []
    for suffix, series, label, axis_bounds, render_mode in targets:
        out_path = os.path.join(out_dir, f"{stem}_{suffix}{ext}")
        _write_mermaid_series_chart(
            out_path,
            tx,
            series,
            label,
            xlabel,
            title,
            y_axis_bounds=axis_bounds,
            render_mode=render_mode,
        )
        written.append(out_path)

    match_path = os.path.join(out_dir, f"{stem}_match{ext}")
    _write_mermaid_match_gantt(
        match_path,
        tx,
        pack["matched"],
        title,
        time_unit="minutes" if "min" in xlabel.lower() else "seconds",
    )
    written.append(match_path)
    return written


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
        help="Deprecated; ignored. Use --sample-interval to control downsampling.",
    )
    parser.add_argument(
        "--raw",
        action="store_true",
        help="Plot every sample as lines (disables time-based downsampling).",
    )
    parser.add_argument(
        "--sample-interval",
        type=float,
        default=0.5,
        help="Seconds between plotted samples in non-raw mode (default: 0.5).",
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
    parser.add_argument(
        "--mermaid",
        nargs="?",
        const="",
        default=None,
        metavar="PATH",
        help="Write Mermaid xychart code (.mmd). Omit PATH for data/mermaid/<input_stem>.mmd; "
        "a bare filename is written under data/mermaid/.",
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

            if not (0 <= vswr <= VSWR_MAX):
                skipped_invalid += 1
                continue

            rows.append((epoch, vswr, motor1, motor2, match))

    return rows, skipped_format, skipped_invalid


def time_downsample(
    t: np.ndarray,
    vswr: np.ndarray,
    motor1: np.ndarray,
    motor2: np.ndarray,
    matched: np.ndarray,
    sample_interval: float,
) -> Tuple[np.ndarray, dict]:
    """Keep real samples, selecting one point approximately every sample_interval."""
    n = len(t)
    if n == 0 or sample_interval <= 0:
        return t, {
            "mode": "raw",
            "vswr": vswr,
            "motor1": motor1,
            "motor2": motor2,
            "matched": matched,
        }

    keep = [0]
    last_t = t[0]
    for i in range(1, n):
        if (t[i] - last_t) >= sample_interval:
            keep.append(i)
            last_t = t[i]
    if keep[-1] != n - 1:
        keep.append(n - 1)

    idx = np.asarray(keep, dtype=np.int64)
    return t[idx], {
        "mode": "sampled",
        "vswr": vswr[idx],
        "motor1": motor1[idx],
        "motor2": motor2[idx],
        "matched": matched[idx],
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
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit(
            "matplotlib is required for static plotting. Install with: python -m pip install matplotlib"
        ) from exc
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

    sample_interval = 0.0 if args.raw else max(0.0, args.sample_interval)
    tx, pack = time_downsample(t_plot, vswr, motor1, motor2, matched, sample_interval)
    if not args.raw:
        print(
            f"Display: {len(tx):,} sampled points "
            f"(every ~{sample_interval:g} s, real VSWR samples)"
        )

    mermaid_out = resolve_mermaid_output(args.mermaid, csv_path)
    if mermaid_out:
        written_paths = write_mermaid_xycharts(
            mermaid_out,
            tx,
            pack,
            xlabel,
            csv_path,
        )
        print("Wrote Mermaid charts:")
        for p in written_paths:
            print(f"  - {p}")
        if not args.interactive and args.html is None:
            return

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

    try:
        import matplotlib.pyplot as plt
        import matplotlib.ticker as ticker
    except ImportError as exc:
        raise SystemExit(
            "matplotlib is required for static plotting. Install with: python -m pip install matplotlib"
        ) from exc

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

    # --- VSWR (real samples; optionally downsampled in time) ---
    ax_vswr.plot(tx, pack["vswr"], color="#08519c", linewidth=0.8, alpha=0.9, label="VSWR")

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

import argparse
import json
import sys
from datetime import datetime
from pathlib import Path
from statistics import median

import visualization as vis


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Capture continuous_scan firmware output, keep refining a rolling 180-degree "
            "point cloud, and stop when the scan looks stable enough."
        )
    )
    parser.add_argument("--port", required=True, help="Serial port, for example COM5")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument(
        "--min-quality",
        type=int,
        default=0,
        help="Drop samples with QUAL lower than this value",
    )
    parser.add_argument(
        "--max-distance",
        type=float,
        default=5000.0,
        help="Drop samples farther than this many millimeters",
    )
    parser.add_argument(
        "--altitude-offset",
        type=float,
        default=0.0,
        help="Adjustment added to ALT in degrees if the scan looks tilted",
    )
    parser.add_argument(
        "--steps-per-rev",
        type=float,
        default=vis.DEFAULT_STEPS_PER_REV,
        help="Stepper steps for one 360-degree platform revolution",
    )
    parser.add_argument(
        "--voxel-size",
        type=float,
        default=0.0,
        help="Final dense export voxel size in millimeters",
    )
    parser.add_argument(
        "--live-voxel-size",
        type=float,
        default=10.0,
        help="Voxel size used for live preview files in millimeters",
    )
    parser.add_argument(
        "--xy-limit",
        type=float,
        default=5000.0,
        help="Discard points outside this X/Y half-width in millimeters",
    )
    parser.add_argument(
        "--z-limit",
        type=float,
        default=5000.0,
        help="Discard points outside this Z half-height in millimeters",
    )
    parser.add_argument(
        "--output-prefix",
        default="continuous_scan",
        help="Base filename used when saving the final clean scan",
    )
    parser.add_argument(
        "--output-dir",
        default="saved_scans",
        help="Folder where the final clean scan is stored",
    )
    parser.add_argument(
        "--live-prefix",
        default="latest_scan",
        help="Base filename used for the rolling live preview files",
    )
    parser.add_argument(
        "--live-dir",
        default="saved_scans/live_preview",
        help="Folder where rolling live preview files are overwritten",
    )
    parser.add_argument(
        "--min-passes",
        type=int,
        default=2,
        help="Minimum completed passes before the scan may be considered clean",
    )
    parser.add_argument(
        "--max-passes",
        type=int,
        default=6,
        help="Maximum passes before saving the best-so-far scan; use 0 for no limit",
    )
    parser.add_argument(
        "--min-ray-pass-support",
        type=int,
        default=2,
        help="How many distinct passes must agree on a ray before it counts as stable",
    )
    parser.add_argument(
        "--clean-coverage-target",
        type=float,
        default=0.55,
        help="Minimum observed ray coverage ratio required to stop automatically",
    )
    parser.add_argument(
        "--clean-stability-target",
        type=float,
        default=0.85,
        help="Minimum stable ray ratio required to stop automatically",
    )
    return parser.parse_args()


def parse_continuous_line(line: str) -> tuple[str, tuple] | None:
    parsed = vis.parse_scan_line(line)
    if parsed is not None:
        return parsed

    parts = line.split(",")
    if not parts:
        return None

    if parts[0] == "MODE" and len(parts) == 2:
        return ("mode", (parts[1],))

    if parts[0] == "FRAME_START" and len(parts) == 5:
        try:
            return (
                "frame_start",
                (
                    int(parts[1]),
                    parts[2],
                    int(parts[3]),
                    int(parts[4]),
                ),
            )
        except ValueError:
            return None

    if parts[0] == "FRAME_DONE" and len(parts) == 4:
        try:
            return (
                "frame_done",
                (
                    int(parts[1]),
                    parts[2],
                    int(parts[3]),
                ),
            )
        except ValueError:
            return None

    return None


def cluster_ray_samples_with_pass(
    samples: list[tuple[float, int, int]],
) -> list[list[tuple[float, int, int]]]:
    if not samples:
        return []

    sorted_samples = sorted(samples, key=lambda sample: sample[0])
    clusters: list[list[tuple[float, int, int]]] = [[sorted_samples[0]]]

    for sample in sorted_samples[1:]:
        last_distance = clusters[-1][-1][0]
        distance_gap = sample[0] - last_distance
        gap_tolerance = max(
            vis.BASE_DISTANCE_TOLERANCE_MM,
            last_distance * vis.BASE_DISTANCE_TOLERANCE_RATIO,
            sample[0] * vis.BASE_DISTANCE_TOLERANCE_RATIO,
        )
        if distance_gap <= gap_tolerance:
            clusters[-1].append(sample)
        else:
            clusters.append([sample])

    return clusters


def choose_primary_cluster_with_pass(
    clusters: list[list[tuple[float, int, int]]],
) -> list[tuple[float, int, int]]:
    return max(
        clusters,
        key=lambda cluster: (
            len(cluster),
            max(sample[1] for sample in cluster),
            -median(sample[0] for sample in cluster),
        ),
    )


def compute_clean_metrics(
    raw_samples_with_pass: list[tuple[int, int, float, float, int]],
    max_azimuth_steps: int,
    min_ray_pass_support: int,
) -> dict[str, float | int]:
    grouped: dict[tuple[int, float], list[tuple[float, int, int]]] = {}
    for pass_index, step_count, altitude_deg, distance_mm, quality in raw_samples_with_pass:
        key = (step_count, altitude_deg)
        bucket = grouped.get(key)
        if bucket is None:
            bucket = []
            grouped[key] = bucket
        bucket.append((distance_mm, quality, pass_index))

    observed_rays = len(grouped)
    stable_rays = 0
    best_pass_support = 0

    for samples in grouped.values():
        primary_cluster = choose_primary_cluster_with_pass(
            cluster_ray_samples_with_pass(samples)
        )
        pass_support = len({sample[2] for sample in primary_cluster})
        best_pass_support = max(best_pass_support, pass_support)
        if pass_support >= min_ray_pass_support:
            stable_rays += 1

    total_possible_rays = max(1, (max_azimuth_steps + 1) * 360)
    coverage_ratio = observed_rays / total_possible_rays
    stable_ray_ratio = stable_rays / observed_rays if observed_rays else 0.0

    return {
        "observed_rays": observed_rays,
        "stable_rays": stable_rays,
        "best_pass_support": best_pass_support,
        "coverage_ratio": coverage_ratio,
        "stable_ray_ratio": stable_ray_ratio,
        "total_possible_rays": total_possible_rays,
    }


def write_live_preview(
    args: argparse.Namespace,
    raw_samples: list[tuple[int, float, float, int]],
    scan_meta: dict[str, float | int | str | bool],
    clean_metrics: dict[str, float | int],
) -> tuple[Path, Path | None, Path]:
    live_dir = Path(args.live_dir)
    live_dir.mkdir(parents=True, exist_ok=True)
    base_path = live_dir / args.live_prefix

    dense_points = vis.build_dense_points(raw_samples, args, args.live_voxel_size)
    dense_csv_path, _ = vis.save_points(
        base_path,
        {(index, 0, 0): point for index, point in enumerate(dense_points)},
    )

    hq_points = vis.build_clean_points(raw_samples, args, args.live_voxel_size)
    hq_csv_path: Path | None = None
    if hq_points:
        hq_csv_path, _ = vis.save_hq_points(base_path, hq_points)

    status_path = base_path.with_name(f"{base_path.name}_status.json")
    payload = {
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "elapsed_runtime": vis.format_elapsed_runtime(),
        "dense_points": len(dense_points),
        "hq_points": len(hq_points),
        "scan_meta": scan_meta,
        "clean_metrics": clean_metrics,
    }
    with status_path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2)
        handle.write("\n")

    return dense_csv_path, hq_csv_path, status_path


def capture_continuous_scan(args: argparse.Namespace) -> int:
    try:
        from serial import Serial, SerialException
    except ImportError:
        print("Missing dependency: pyserial", file=sys.stderr)
        print("Install it with: pip install pyserial", file=sys.stderr)
        return 1

    try:
        vis.print_status(f"Opening {args.port} at {args.baud} baud...")
        serial_port = Serial(args.port, args.baud, timeout=0.1)
        vis.print_status("Serial port opened. Waiting for continuous scan stream...")
    except SerialException as exc:
        print(f"Could not open {args.port}: {exc}", file=sys.stderr)
        return 1

    raw_samples: list[tuple[int, float, float, int]] = []
    raw_samples_with_pass: list[tuple[int, int, float, float, int]] = []
    accepted_points = 0
    passes_completed = 0
    current_pass_index: int | None = None
    current_pass_direction = "unknown"
    current_pass_points = 0
    preview_paths_reported = False
    warned_mid_pass_attach = False
    scan_meta: dict[str, float | int | str | bool] = {
        "mode": "continuous",
        "max_azimuth_deg": vis.DEFAULT_MAX_AZIMUTH_DEG,
        "max_azimuth_steps": vis.DEFAULT_MAX_AZIMUTH_STEPS,
        "packets_per_sweep": vis.DEFAULT_PACKETS_PER_SWEEP,
        "step_delay_us": vis.DEFAULT_STEP_DELAY_US,
        "step_settle_ms": vis.DEFAULT_STEP_SETTLE_MS,
        "sweep_timeout_ms": vis.DEFAULT_SWEEP_TIMEOUT_MS,
        "byte_timeout_ms": vis.DEFAULT_BYTE_TIMEOUT_MS,
        "metadata_source": "defaults",
        "config_source": "defaults",
        "complete": False,
    }

    try:
        while True:
            try:
                raw = serial_port.readline()
            except (SerialException, OSError) as exc:
                scan_meta["completion_reason"] = "serial_error"
                scan_meta["interruption_message"] = str(exc)
                print(f"Serial error while reading {args.port}: {exc}", file=sys.stderr)
                return vis.finalize_scan_outputs(
                    args,
                    raw_samples,
                    accepted_points,
                    scan_meta,
                    partial=True,
                )

            if not raw:
                continue

            line = raw.decode("utf-8", errors="ignore").strip()
            parsed = parse_continuous_line(line)
            if parsed is None:
                continue

            kind, payload = parsed

            if kind == "mode":
                scan_meta["firmware_mode"] = payload[0]
                vis.print_status(f"Firmware mode: {payload[0]}")
                continue

            if kind == "meta":
                scan_meta["max_azimuth_deg"], scan_meta["max_azimuth_steps"] = payload
                scan_meta["metadata_source"] = "firmware"
                vis.print_status(
                    f"Continuous scan metadata received: "
                    f"{scan_meta['max_azimuth_deg']:.1f} deg sweep, "
                    f"{scan_meta['max_azimuth_steps']} step positions"
                )
                continue

            if kind == "config":
                (
                    scan_meta["packets_per_sweep"],
                    scan_meta["step_delay_us"],
                    scan_meta["step_settle_ms"],
                    scan_meta["sweep_timeout_ms"],
                    scan_meta["byte_timeout_ms"],
                ) = payload
                scan_meta["config_source"] = "firmware"
                vis.print_status(
                    "Continuous capture config received: "
                    f"{scan_meta['packets_per_sweep']} packets/sweep, "
                    f"step delay {scan_meta['step_delay_us']} us, "
                    f"settle {scan_meta['step_settle_ms']} ms"
                )
                continue

            if kind == "frame_start":
                frame_index, direction, start_step, end_step = payload
                current_pass_index = frame_index
                current_pass_direction = direction
                current_pass_points = 0
                vis.print_status(
                    f"Pass {frame_index} started ({direction}, {start_step}->{end_step})"
                )
                continue

            if kind == "frame_done":
                frame_index, direction, final_step = payload
                passes_completed += 1
                scan_meta["passes_completed"] = passes_completed
                scan_meta["last_frame_index"] = frame_index
                scan_meta["last_frame_direction"] = direction
                scan_meta["last_frame_final_step"] = final_step

                vis.print_status(
                    f"Pass {frame_index} complete ({direction}) with "
                    f"{current_pass_points} accepted points in this pass"
                )

                clean_metrics = compute_clean_metrics(
                    raw_samples_with_pass,
                    int(scan_meta["max_azimuth_steps"]),
                    args.min_ray_pass_support,
                )
                scan_meta["clean_metrics"] = clean_metrics

                vis.print_status("Updating rolling live preview files...")
                dense_csv_path, hq_csv_path, status_path = write_live_preview(
                    args,
                    raw_samples,
                    scan_meta,
                    clean_metrics,
                )
                if not preview_paths_reported:
                    print(f"Live dense preview: {dense_csv_path}")
                    if hq_csv_path is not None:
                        print(f"Live HQ preview: {hq_csv_path}")
                    print(f"Live status: {status_path}")
                    preview_paths_reported = True

                vis.print_status(
                    "Clean scan check: "
                    f"coverage {clean_metrics['coverage_ratio'] * 100.0:5.1f}%, "
                    f"stable rays {clean_metrics['stable_ray_ratio'] * 100.0:5.1f}%"
                )

                clean_enough = (
                    passes_completed >= args.min_passes
                    and clean_metrics["coverage_ratio"] >= args.clean_coverage_target
                    and clean_metrics["stable_ray_ratio"] >= args.clean_stability_target
                )
                if clean_enough:
                    scan_meta["complete"] = True
                    scan_meta["completion_reason"] = "clean_threshold_met"
                    vis.print_status("Scan is clean enough. Saving final outputs...")
                    return vis.finalize_scan_outputs(
                        args,
                        raw_samples,
                        accepted_points,
                        scan_meta,
                        partial=False,
                    )

                if args.max_passes > 0 and passes_completed >= args.max_passes:
                    scan_meta["complete"] = False
                    scan_meta["completion_reason"] = "max_passes_reached"
                    vis.print_status(
                        "Reached the pass limit before the clean target. "
                        "Saving the best-so-far scan..."
                    )
                    return vis.finalize_scan_outputs(
                        args,
                        raw_samples,
                        accepted_points,
                        scan_meta,
                        partial=False,
                    )

                current_pass_index = None
                current_pass_direction = "unknown"
                current_pass_points = 0
                continue

            if kind == "done":
                scan_meta["complete"] = True
                scan_meta["completion_reason"] = "firmware_done"
                vis.print_status(
                    "Received DONE from firmware. Saving current accumulated outputs..."
                )
                return vis.finalize_scan_outputs(
                    args,
                    raw_samples,
                    accepted_points,
                    scan_meta,
                    partial=False,
                )

            step_count, altitude_deg, distance_mm, quality = payload
            if quality < args.min_quality:
                continue
            if distance_mm <= 0 or distance_mm > args.max_distance:
                continue

            if current_pass_index is None:
                current_pass_index = passes_completed
                if not warned_mid_pass_attach:
                    vis.print_status(
                        "Point stream detected before a pass marker; "
                        "treating the current sweep as a synthetic pass"
                    )
                    warned_mid_pass_attach = True

            azimuth_deg = vis.step_to_azimuth_deg(step_count, args.steps_per_rev)
            x, y, z = vis.point_from_angles(
                azimuth_deg,
                altitude_deg + args.altitude_offset,
                distance_mm,
            )
            if abs(x) > args.xy_limit or abs(y) > args.xy_limit or abs(z) > args.z_limit:
                continue

            accepted_points += 1
            current_pass_points += 1
            if accepted_points % vis.PROGRESS_PRINT_INTERVAL == 0:
                print(f"Captured {accepted_points} raw points so far...")

            raw_samples.append((step_count, altitude_deg, distance_mm, quality))
            raw_samples_with_pass.append(
                (current_pass_index, step_count, altitude_deg, distance_mm, quality)
            )
    except KeyboardInterrupt:
        scan_meta["complete"] = False
        scan_meta["completion_reason"] = "keyboard_interrupt"
        print("Continuous capture interrupted by user.", file=sys.stderr)
        return vis.finalize_scan_outputs(
            args,
            raw_samples,
            accepted_points,
            scan_meta,
            partial=True,
        )
    finally:
        serial_port.close()


def main() -> int:
    args = parse_args()
    return capture_continuous_scan(args)


if __name__ == "__main__":
    raise SystemExit(main())

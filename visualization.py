import argparse
import csv
import json
import math
import re
import sys
import time
from statistics import median
from datetime import datetime
from pathlib import Path


DEFAULT_STEPS_PER_REV = 810.0
DEFAULT_MAX_AZIMUTH_DEG = 180.0
DEFAULT_MAX_AZIMUTH_STEPS = int(DEFAULT_STEPS_PER_REV / 2.0)
DEFAULT_PACKETS_PER_SWEEP = 1080
DEFAULT_STEP_DELAY_US = 3700
DEFAULT_STEP_SETTLE_MS = 10
DEFAULT_SWEEP_TIMEOUT_MS = 15000
DEFAULT_BYTE_TIMEOUT_MS = 20
BASE_DISTANCE_TOLERANCE_MM = 45.0
BASE_DISTANCE_TOLERANCE_RATIO = 0.05
DENSE_MIN_CLUSTER_SAMPLES = 2
DENSE_DISTANCE_SHRINK_RATIO = 0.25
SWEEP_PROGRESS_BUCKETS = 20
PROGRESS_PRINT_INTERVAL = 50000
TIMESTAMP_PATTERN = re.compile(r"^(?P<prefix>.+)_(?P<date>\d{8})_(?P<time>\d{6})_(?P<micros>\d{6})(?P<suffix>.*)$")
PROGRAM_START_MONOTONIC = time.monotonic()
PointRecord = tuple[float, float, float, float, int]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture merged_scan output and save point cloud CSV files."
    )
    parser.add_argument("--port", help="Serial port, for example COM5")
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
        default=DEFAULT_STEPS_PER_REV,
        help="Stepper steps for one 360-degree platform revolution",
    )
    parser.add_argument(
        "--voxel-size",
        type=float,
        default=0.0,
        help="Merge nearby points into 3D voxels of this size in millimeters",
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
        default="room_scan",
        help="Base filename used when saving completed scans",
    )
    parser.add_argument(
        "--output-dir",
        default="saved_scans",
        help="Folder where completed scan files are stored",
    )
    parser.add_argument(
        "--organize-only",
        action="store_true",
        help="Reorganize existing scans in the output folder and exit",
    )
    args = parser.parse_args()
    if args.organize_only:
        return args
    if not args.port:
        parser.error("--port is required unless --organize-only is used")
    return args


def print_status(message: str) -> None:
    timestamp = datetime.now().strftime("%H:%M:%S")
    print(f"[{timestamp} +{format_elapsed_runtime()}] {message}")


def format_elapsed_runtime() -> str:
    elapsed_seconds = int(time.monotonic() - PROGRAM_START_MONOTONIC)
    hours, remainder = divmod(elapsed_seconds, 3600)
    minutes, seconds = divmod(remainder, 60)
    if hours:
        return f"{hours:02d}:{minutes:02d}:{seconds:02d}"
    return f"{minutes:02d}:{seconds:02d}"


def step_to_azimuth_deg(step_count: int, steps_per_rev: float) -> float:
    return (step_count * 360.0) / steps_per_rev


def point_from_angles(
    azimuth_deg: float, altitude_deg: float, distance_mm: float
) -> tuple[float, float, float]:
    azimuth = math.radians(azimuth_deg)
    altitude = math.radians(altitude_deg)

    horizontal_radius = distance_mm * math.cos(altitude)
    x = horizontal_radius * math.cos(azimuth)
    y = horizontal_radius * math.sin(azimuth)
    z = distance_mm * math.sin(altitude)
    return x, y, z


def voxel_key(x: float, y: float, z: float, voxel_size: float) -> tuple[int, int, int]:
    return (
        math.floor(x / voxel_size),
        math.floor(y / voxel_size),
        math.floor(z / voxel_size),
    )


def parse_scan_line(line: str) -> tuple[str, tuple] | None:
    parts = line.split(",")
    if not parts:
        return None

    if parts[0] == "P" and len(parts) == 5:
        try:
            return (
                "point",
                (
                    int(parts[1]),
                    float(parts[2]),
                    float(parts[3]),
                    int(parts[4]),
                ),
            )
        except ValueError:
            return None

    if parts[0] == "META" and len(parts) == 3:
        try:
            return ("meta", (float(parts[1]), int(parts[2])))
        except ValueError:
            return None

    if parts[0] == "CFG" and len(parts) == 6:
        try:
            return (
                "config",
                (
                    int(parts[1]),
                    int(parts[2]),
                    int(parts[3]),
                    int(parts[4]),
                    int(parts[5]),
                ),
            )
        except ValueError:
            return None

    if parts[0] == "DONE" and len(parts) == 2:
        try:
            return ("done", (int(parts[1]),))
        except ValueError:
            return None

    return None


def ensure_output_base(output_dir: str, output_prefix: str) -> Path:
    output_path = Path(output_dir)
    timestamp = datetime.now()
    scan_dir = output_path / timestamp.strftime("%Y-%m-%d") / timestamp.strftime("%H-%M-%S-%f")
    scan_dir.mkdir(parents=True, exist_ok=True)
    scan_stamp = timestamp.strftime("%Y%m%d_%H%M%S_%f")
    return scan_dir / f"{output_prefix}_{scan_stamp}"


def parse_timestamped_stem(stem: str) -> tuple[str, datetime, str] | None:
    match = TIMESTAMP_PATTERN.match(stem)
    if match is None:
        return None

    try:
        timestamp = datetime.strptime(
            f"{match.group('date')}_{match.group('time')}_{match.group('micros')}",
            "%Y%m%d_%H%M%S_%f",
        )
    except ValueError:
        return None

    return match.group("prefix"), timestamp, match.group("suffix")


def scan_dir_from_timestamp(output_dir: Path, timestamp: datetime) -> Path:
    return output_dir / timestamp.strftime("%Y-%m-%d") / timestamp.strftime("%H-%M-%S-%f")


def organize_existing_scans(output_dir: str) -> int:
    output_path = Path(output_dir)
    if not output_path.exists():
        return 0

    moved_files = 0
    for file_path in sorted(output_path.glob("*")):
        if not file_path.is_file():
            continue

        parsed = parse_timestamped_stem(file_path.stem)
        if parsed is None:
            continue

        _, timestamp, _ = parsed
        target_dir = scan_dir_from_timestamp(output_path, timestamp)
        target_dir.mkdir(parents=True, exist_ok=True)
        target_path = target_dir / file_path.name

        if file_path.resolve() == target_path.resolve():
            continue

        if target_path.exists():
            continue

        file_path.rename(target_path)
        moved_files += 1

    return moved_files


def save_points(
    base_path: Path,
    points: list[PointRecord],
) -> Path:
    return save_point_csv(base_path, "points", points)


def save_scan_settings(
    base_path: Path,
    args: argparse.Namespace,
    raw_points: int,
    exported_points: int,
    hq_points: int | None = None,
    scan_meta: dict[str, float | int | str | bool] | None = None,
) -> Path:
    settings_path = base_path.with_name(f"{base_path.name}_settings.json")
    payload = {
        "port": args.port,
        "baud": args.baud,
        "min_quality": args.min_quality,
        "max_distance": args.max_distance,
        "altitude_offset": args.altitude_offset,
        "steps_per_rev": args.steps_per_rev,
        "voxel_size": args.voxel_size,
        "xy_limit": args.xy_limit,
        "z_limit": args.z_limit,
        "output_prefix": args.output_prefix,
        "raw_points": raw_points,
        "exported_points": exported_points,
    }
    if hq_points is not None:
        payload["hq_points"] = hq_points
    if scan_meta:
        payload["scan_meta"] = scan_meta

    with settings_path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2)
        handle.write("\n")

    return settings_path


def save_hq_points(
    base_path: Path,
    points: list[PointRecord],
) -> Path:
    return save_point_csv(base_path, "hq_points", points)


def save_raw_points(
    base_path: Path,
    points: list[PointRecord],
) -> Path:
    return save_point_csv(base_path, "raw_points", points)


def save_point_csv(
    base_path: Path,
    suffix: str,
    points: list[PointRecord],
) -> Path:
    csv_path = base_path.with_name(f"{base_path.name}_{suffix}.csv")
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["x_mm", "y_mm", "z_mm", "distance_mm", "quality"])
        writer.writerows(points)
    return csv_path


def merge_points_into_voxels(
    points: list[PointRecord],
    voxel_size: float,
) -> list[PointRecord]:
    if voxel_size <= 0:
        return points

    grouped: dict[tuple[int, int, int], list[PointRecord]] = {}
    for point in points:
        key = voxel_key(point[0], point[1], point[2], voxel_size)
        bucket = grouped.get(key)
        if bucket is None:
            bucket = []
            grouped[key] = bucket
        bucket.append(point)

    merged_points: list[PointRecord] = []
    for bucket in grouped.values():
        count = len(bucket)
        merged_points.append(
            (
                sum(point[0] for point in bucket) / count,
                sum(point[1] for point in bucket) / count,
                sum(point[2] for point in bucket) / count,
                sum(point[3] for point in bucket) / count,
                max(point[4] for point in bucket),
            )
        )

    return merged_points


def cluster_ray_samples(
    samples: list[tuple[float, int]],
) -> list[list[tuple[float, int]]]:
    if not samples:
        return []

    sorted_samples = sorted(samples, key=lambda sample: sample[0])
    clusters: list[list[tuple[float, int]]] = [[sorted_samples[0]]]

    for sample in sorted_samples[1:]:
        last_distance = clusters[-1][-1][0]
        distance_gap = sample[0] - last_distance
        gap_tolerance = max(
            BASE_DISTANCE_TOLERANCE_MM,
            last_distance * BASE_DISTANCE_TOLERANCE_RATIO,
            sample[0] * BASE_DISTANCE_TOLERANCE_RATIO,
        )
        if distance_gap <= gap_tolerance:
            clusters[-1].append(sample)
        else:
            clusters.append([sample])

    return clusters


def choose_primary_cluster(
    clusters: list[list[tuple[float, int]]],
) -> list[tuple[float, int]]:
    return max(
        clusters,
        key=lambda cluster: (
            len(cluster),
            max(sample[1] for sample in cluster),
            -median(sample[0] for sample in cluster),
        ),
    )


def representative_point_from_cluster(
    step_count: int,
    altitude_deg: float,
    cluster: list[tuple[float, int]],
    args: argparse.Namespace,
) -> PointRecord | None:
    representative_distance = median(sample[0] for sample in cluster)
    representative_quality = max(sample[1] for sample in cluster)
    azimuth_deg = step_to_azimuth_deg(step_count, args.steps_per_rev)
    x, y, z = point_from_angles(
        azimuth_deg,
        altitude_deg + args.altitude_offset,
        representative_distance,
    )

    if abs(x) > args.xy_limit or abs(y) > args.xy_limit or abs(z) > args.z_limit:
        return None

    return (x, y, z, representative_distance, representative_quality)


def point_from_distance_and_quality(
    step_count: int,
    altitude_deg: float,
    distance_mm: float,
    quality: int,
    args: argparse.Namespace,
) -> PointRecord | None:
    azimuth_deg = step_to_azimuth_deg(step_count, args.steps_per_rev)
    x, y, z = point_from_angles(
        azimuth_deg,
        altitude_deg + args.altitude_offset,
        distance_mm,
    )

    if abs(x) > args.xy_limit or abs(y) > args.xy_limit or abs(z) > args.z_limit:
        return None

    return (x, y, z, distance_mm, quality)


def group_raw_samples_by_ray(
    raw_samples: list[tuple[int, float, float, int]],
) -> dict[tuple[int, float], list[tuple[float, int]]]:
    grouped: dict[tuple[int, float], list[tuple[float, int]]] = {}
    for step_count, altitude_deg, distance_mm, quality in raw_samples:
        key = (step_count, altitude_deg)
        bucket = grouped.get(key)
        if bucket is None:
            bucket = []
            grouped[key] = bucket
        bucket.append((distance_mm, quality))

    return grouped


def build_clean_points(
    raw_samples: list[tuple[int, float, float, int]],
    args: argparse.Namespace,
    voxel_size: float,
) -> list[PointRecord]:
    cleaned_points: list[PointRecord] = []
    for (step_count, altitude_deg), samples in group_raw_samples_by_ray(raw_samples).items():
        clusters = cluster_ray_samples(samples)
        primary_cluster = choose_primary_cluster(clusters)
        representative = representative_point_from_cluster(
            step_count,
            altitude_deg,
            primary_cluster,
            args,
        )
        if representative is not None:
            cleaned_points.append(representative)

    return merge_points_into_voxels(cleaned_points, voxel_size)


def build_dense_points(
    raw_samples: list[tuple[int, float, float, int]],
    args: argparse.Namespace,
    voxel_size: float,
) -> list[PointRecord]:
    dense_points: list[PointRecord] = []
    for (step_count, altitude_deg), samples in group_raw_samples_by_ray(raw_samples).items():
        clusters = cluster_ray_samples(samples)
        primary_cluster = choose_primary_cluster(clusters)
        kept_clusters: list[list[tuple[float, int]]] = [primary_cluster]
        for cluster in clusters:
            if cluster is primary_cluster:
                continue
            if len(cluster) >= DENSE_MIN_CLUSTER_SAMPLES:
                kept_clusters.append(cluster)

        for cluster in kept_clusters:
            representative_distance = median(sample[0] for sample in cluster)
            for sample_distance, sample_quality in cluster:
                adjusted_distance = representative_distance + (
                    (sample_distance - representative_distance) * DENSE_DISTANCE_SHRINK_RATIO
                )
                point = point_from_distance_and_quality(
                    step_count,
                    altitude_deg,
                    adjusted_distance,
                    sample_quality,
                    args,
                )
                if point is not None:
                    dense_points.append(point)

    return merge_points_into_voxels(dense_points, voxel_size)


def build_raw_points(
    raw_samples: list[tuple[int, float, float, int]],
    args: argparse.Namespace,
) -> list[PointRecord]:
    raw_points: list[PointRecord] = []
    for step_count, altitude_deg, distance_mm, quality in raw_samples:
        azimuth_deg = step_to_azimuth_deg(step_count, args.steps_per_rev)
        x, y, z = point_from_angles(
            azimuth_deg,
            altitude_deg + args.altitude_offset,
            distance_mm,
        )
        raw_points.append((x, y, z, distance_mm, quality))

    return raw_points


def finalize_scan_outputs(
    args: argparse.Namespace,
    raw_samples: list[tuple[int, float, float, int]],
    accepted_points: int,
    scan_meta: dict[str, float | int | str | bool],
    *,
    partial: bool,
) -> int:
    if not raw_samples:
        if partial:
            print("Capture stopped before any valid points were saved.", file=sys.stderr)
        else:
            print("Scan finished, but no valid points were captured.", file=sys.stderr)
        return 1

    output_prefix = args.output_prefix if not partial else f"{args.output_prefix}_partial"
    base_path = ensure_output_base(args.output_dir, output_prefix)
    print_status("Preparing raw point cloud...")
    raw_points = build_raw_points(raw_samples, args)
    print_status("Saving raw point cloud CSV...")
    raw_csv_path = save_raw_points(base_path, raw_points)
    print_status("Building dense outline point cloud...")
    points = build_dense_points(raw_samples, args, args.voxel_size)
    if not points:
        print("Scan finished, but all captured points were filtered out.", file=sys.stderr)
        return 1

    print_status("Saving dense outline point cloud CSV...")
    csv_path = save_points(base_path, points)
    print_status("Building HQ point cloud...")
    hq_points = build_clean_points(raw_samples, args, 0.0)
    hq_csv_path = None
    if hq_points:
        print_status("Saving HQ point cloud CSV...")
        hq_csv_path = save_hq_points(base_path, hq_points)

    print_status("Writing scan settings...")
    settings_path = save_scan_settings(
        base_path,
        args,
        accepted_points,
        len(points),
        len(hq_points) if hq_points else None,
        scan_meta,
    )

    if partial:
        print("Saved partial scan after capture interruption.", file=sys.stderr)
    print(f"Saved raw scan to {raw_csv_path}")
    print(f"Raw scan kept {len(raw_points)} accepted points")
    print(f"Saved scan to {csv_path}")
    print(f"Accepted {accepted_points} raw points")
    print(f"Exported {len(points)} dense outline points")
    print(f"Saved scan settings to {settings_path}")
    if hq_csv_path is not None:
        print(f"Saved HQ scan to {hq_csv_path}")
        print(f"HQ scan kept {len(hq_points)} primary-surface points")
    print_status(f"Total runtime: {format_elapsed_runtime()}")

    return 0 if not partial else 1


def maybe_report_sweep_progress(
    step_count: int,
    scan_meta: dict[str, float | int | str | bool],
    accepted_points: int,
    last_step_seen: int | None,
    last_progress_bucket: int,
) -> tuple[int | None, int]:
    if step_count == last_step_seen:
        return last_step_seen, last_progress_bucket

    max_steps = scan_meta.get("max_azimuth_steps")
    if not isinstance(max_steps, int) or max_steps <= 0:
        return step_count, last_progress_bucket

    bucket = min(
        SWEEP_PROGRESS_BUCKETS,
        int((step_count * SWEEP_PROGRESS_BUCKETS) / max_steps),
    )
    if bucket > last_progress_bucket:
        percent = (step_count * 100.0) / max_steps
        print_status(
            f"Sweep progress: {percent:5.1f}% ({step_count}/{max_steps} steps), "
            f"accepted {accepted_points} points"
        )
        last_progress_bucket = bucket

    return step_count, last_progress_bucket


def capture_from_serial(args: argparse.Namespace) -> int:
    try:
        from serial import Serial, SerialException
    except ImportError:
        print("Missing dependency: pyserial", file=sys.stderr)
        print("Install it with: pip install pyserial", file=sys.stderr)
        return 1

    try:
        print_status(f"Opening {args.port} at {args.baud} baud...")
        serial_port = Serial(args.port, args.baud, timeout=0.1)
        print_status("Serial port opened. Waiting for scan stream...")
    except SerialException as exc:
        print(f"Could not open {args.port}: {exc}", file=sys.stderr)
        return 1

    raw_samples: list[tuple[int, float, float, int]] = []
    accepted_points = 0
    scan_meta: dict[str, float | int | str | bool] = {
        "max_azimuth_deg": DEFAULT_MAX_AZIMUTH_DEG,
        "max_azimuth_steps": DEFAULT_MAX_AZIMUTH_STEPS,
        "packets_per_sweep": DEFAULT_PACKETS_PER_SWEEP,
        "step_delay_us": DEFAULT_STEP_DELAY_US,
        "step_settle_ms": DEFAULT_STEP_SETTLE_MS,
        "sweep_timeout_ms": DEFAULT_SWEEP_TIMEOUT_MS,
        "byte_timeout_ms": DEFAULT_BYTE_TIMEOUT_MS,
        "metadata_source": "defaults",
        "config_source": "defaults",
    }
    last_step_seen: int | None = None
    last_progress_bucket = -1
    warned_mid_scan_attach = False

    try:
        while True:
            try:
                raw = serial_port.readline()
            except (SerialException, OSError) as exc:
                scan_meta["complete"] = False
                scan_meta["completion_reason"] = "serial_error"
                scan_meta["interruption_message"] = str(exc)
                print(f"Serial error while reading {args.port}: {exc}", file=sys.stderr)
                return finalize_scan_outputs(
                    args,
                    raw_samples,
                    accepted_points,
                    scan_meta,
                    partial=True,
                )
            if not raw:
                continue

            line = raw.decode("utf-8", errors="ignore").strip()
            parsed = parse_scan_line(line)
            if parsed is None:
                continue

            kind, payload = parsed
            if kind == "meta":
                scan_meta["max_azimuth_deg"], scan_meta["max_azimuth_steps"] = payload
                scan_meta["metadata_source"] = "firmware"
                print_status(
                    f"Scan metadata received: "
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
                print_status(
                    "Capture config received: "
                    f"{scan_meta['packets_per_sweep']} packets/sweep, "
                    f"step delay {scan_meta['step_delay_us']} us, "
                    f"settle {scan_meta['step_settle_ms']} ms"
                )
                continue

            if kind == "done":
                scan_meta["complete"] = True
                scan_meta["completion_reason"] = "done"
                print_status("Firmware reported scan complete. Finalizing outputs...")
                return finalize_scan_outputs(
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

            if not warned_mid_scan_attach and scan_meta.get("metadata_source") != "firmware":
                print_status(
                    "Point stream detected before firmware metadata; "
                    f"using defaults ({DEFAULT_MAX_AZIMUTH_DEG:.1f} deg, "
                    f"{DEFAULT_MAX_AZIMUTH_STEPS} steps) for progress and settings"
                )
                warned_mid_scan_attach = True

            last_step_seen, last_progress_bucket = maybe_report_sweep_progress(
                step_count,
                scan_meta,
                accepted_points + 1,
                last_step_seen,
                last_progress_bucket,
            )

            azimuth_deg = step_to_azimuth_deg(step_count, args.steps_per_rev)
            x, y, z = point_from_angles(
                azimuth_deg,
                altitude_deg + args.altitude_offset,
                distance_mm,
            )

            if abs(x) > args.xy_limit or abs(y) > args.xy_limit or abs(z) > args.z_limit:
                continue

            accepted_points += 1
            if accepted_points % PROGRESS_PRINT_INTERVAL == 0:
                print(f"Captured {accepted_points} raw points so far...")
            raw_samples.append((step_count, altitude_deg, distance_mm, quality))
    except KeyboardInterrupt:
        scan_meta["complete"] = False
        scan_meta["completion_reason"] = "keyboard_interrupt"
        print("Capture interrupted by user.", file=sys.stderr)
        return finalize_scan_outputs(
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
    moved_files = organize_existing_scans(args.output_dir)
    if moved_files:
        print(f"Organized {moved_files} existing scan file(s) in {args.output_dir}")
    if args.organize_only:
        return 0
    return capture_from_serial(args)


if __name__ == "__main__":
    raise SystemExit(main())

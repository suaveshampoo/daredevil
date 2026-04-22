import argparse
import csv
import math
from pathlib import Path


DEFAULT_NEIGHBOR_RADIUS_MM = 80.0
DEFAULT_MAX_NEIGHBORS = 24
DEFAULT_SAMPLE_COUNT = 200
DEFAULT_MIN_NEIGHBORS = 6
DEFAULT_COARSE_VOXEL_MM = 100.0
DEFAULT_COVERAGE_TARGET = 7000


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Benchmark saved scan CSV files with geometry and coverage-aware metrics."
    )
    parser.add_argument(
        "input_path",
        help="Path to a scan CSV file or a folder containing scan CSV files",
    )
    parser.add_argument(
        "--neighbor-radius",
        type=float,
        default=DEFAULT_NEIGHBOR_RADIUS_MM,
        help="Neighbor radius in millimeters for local geometry checks",
    )
    parser.add_argument(
        "--max-neighbors",
        type=int,
        default=DEFAULT_MAX_NEIGHBORS,
        help="Maximum neighbors to use per sampled point",
    )
    parser.add_argument(
        "--sample-count",
        type=int,
        default=DEFAULT_SAMPLE_COUNT,
        help="How many evenly spaced points to evaluate",
    )
    parser.add_argument(
        "--min-neighbors",
        type=int,
        default=DEFAULT_MIN_NEIGHBORS,
        help="Minimum neighbors required to treat a sample as locally supported",
    )
    parser.add_argument(
        "--coarse-voxel-size",
        type=float,
        default=DEFAULT_COARSE_VOXEL_MM,
        help="Voxel size in millimeters used to estimate useful scene coverage",
    )
    parser.add_argument(
        "--coverage-target",
        type=int,
        default=DEFAULT_COVERAGE_TARGET,
        help="Supported coarse voxels needed to saturate coverage score at 100",
    )
    return parser.parse_args()


def load_points(csv_path: Path) -> list[tuple[float, float, float, float, int]]:
    points: list[tuple[float, float, float, float, int]] = []
    with csv_path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            points.append(
                (
                    float(row["x_mm"]),
                    float(row["y_mm"]),
                    float(row["z_mm"]),
                    float(row["distance_mm"]),
                    int(float(row["quality"])),
                )
            )
    return points


def percentile(sorted_values: list[float], pct: float) -> float:
    if not sorted_values:
        return 0.0
    if len(sorted_values) == 1:
        return sorted_values[0]

    position = pct * (len(sorted_values) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return sorted_values[lower]

    fraction = position - lower
    return sorted_values[lower] * (1.0 - fraction) + sorted_values[upper] * fraction


def voxel_key(
    point: tuple[float, float, float],
    voxel_size: float,
) -> tuple[int, int, int]:
    return (
        math.floor(point[0] / voxel_size),
        math.floor(point[1] / voxel_size),
        math.floor(point[2] / voxel_size),
    )


def axis_spread(values: list[float]) -> float:
    return max(values) - min(values)


def legacy_thickness(points: list[tuple[float, float, float]]) -> float:
    spreads = [
        axis_spread([point[0] for point in points]),
        axis_spread([point[1] for point in points]),
        axis_spread([point[2] for point in points]),
    ]
    spreads.sort()
    return spreads[0]


def eigenvalues_symmetric_3x3(
    xx: float,
    yy: float,
    zz: float,
    xy: float,
    yz: float,
    xz: float,
) -> tuple[float, float, float]:
    p1 = xy * xy + yz * yz + xz * xz
    if p1 == 0.0:
        return tuple(sorted((xx, yy, zz)))

    q = (xx + yy + zz) / 3.0
    xx_q = xx - q
    yy_q = yy - q
    zz_q = zz - q
    p2 = xx_q * xx_q + yy_q * yy_q + zz_q * zz_q + 2.0 * p1
    p = math.sqrt(max(p2 / 6.0, 0.0))
    if p == 0.0:
        return q, q, q

    b_xx = xx_q / p
    b_yy = yy_q / p
    b_zz = zz_q / p
    b_xy = xy / p
    b_yz = yz / p
    b_xz = xz / p
    det_b = (
        b_xx * (b_yy * b_zz - b_yz * b_yz)
        - b_xy * (b_xy * b_zz - b_yz * b_xz)
        + b_xz * (b_xy * b_yz - b_yy * b_xz)
    )
    r = max(-1.0, min(1.0, det_b / 2.0))
    phi = math.acos(r) / 3.0

    eig1 = q + 2.0 * p * math.cos(phi)
    eig3 = q + 2.0 * p * math.cos(phi + (2.0 * math.pi / 3.0))
    eig2 = 3.0 * q - eig1 - eig3
    return tuple(sorted((eig1, eig2, eig3)))


def plane_rmse(points: list[tuple[float, float, float]]) -> float:
    count = len(points)
    mean_x = sum(point[0] for point in points) / count
    mean_y = sum(point[1] for point in points) / count
    mean_z = sum(point[2] for point in points) / count

    xx = 0.0
    yy = 0.0
    zz = 0.0
    xy = 0.0
    yz = 0.0
    xz = 0.0
    for x, y, z in points:
        dx = x - mean_x
        dy = y - mean_y
        dz = z - mean_z
        xx += dx * dx
        yy += dy * dy
        zz += dz * dz
        xy += dx * dy
        yz += dy * dz
        xz += dx * dz

    scale = 1.0 / count
    smallest_eigenvalue, _, _ = eigenvalues_symmetric_3x3(
        xx * scale,
        yy * scale,
        zz * scale,
        xy * scale,
        yz * scale,
        xz * scale,
    )
    return math.sqrt(max(smallest_eigenvalue, 0.0))


def build_spatial_index(
    xyz_points: list[tuple[float, float, float]],
    cell_size: float,
) -> dict[tuple[int, int, int], list[int]]:
    grid: dict[tuple[int, int, int], list[int]] = {}
    for index, point in enumerate(xyz_points):
        key = voxel_key(point, cell_size)
        bucket = grid.get(key)
        if bucket is None:
            bucket = []
            grid[key] = bucket
        bucket.append(index)
    return grid


def find_neighbors(
    index: int,
    xyz_points: list[tuple[float, float, float]],
    grid: dict[tuple[int, int, int], list[int]],
    radius_mm: float,
    max_neighbors: int,
) -> tuple[list[tuple[float, float, float]], float | None]:
    center = xyz_points[index]
    center_key = voxel_key(center, radius_mm)
    radius_sq = radius_mm * radius_mm
    candidates: list[tuple[float, tuple[float, float, float]]] = []

    for offset_x in (-1, 0, 1):
        for offset_y in (-1, 0, 1):
            for offset_z in (-1, 0, 1):
                neighbor_key = (
                    center_key[0] + offset_x,
                    center_key[1] + offset_y,
                    center_key[2] + offset_z,
                )
                for neighbor_index in grid.get(neighbor_key, []):
                    if neighbor_index == index:
                        continue
                    point = xyz_points[neighbor_index]
                    dx = center[0] - point[0]
                    dy = center[1] - point[1]
                    dz = center[2] - point[2]
                    dist_sq = dx * dx + dy * dy + dz * dz
                    if dist_sq <= radius_sq:
                        candidates.append((dist_sq, point))

    candidates.sort(key=lambda item: item[0])
    nearest_distance = None
    if candidates:
        nearest_distance = math.sqrt(candidates[0][0])
    return [point for _, point in candidates[:max_neighbors]], nearest_distance


def clamp_score(value: float) -> float:
    return max(0.0, min(100.0, value))


def benchmark_file(csv_path: Path, args: argparse.Namespace) -> dict[str, float | int | Path]:
    points = load_points(csv_path)
    if len(points) < 20:
        raise ValueError(f"Not enough points in {csv_path} to benchmark.")

    xyz_points = [(point[0], point[1], point[2]) for point in points]
    grid = build_spatial_index(xyz_points, args.neighbor_radius)
    coarse_grid = build_spatial_index(xyz_points, args.coarse_voxel_size)
    sample_step = max(1, len(points) // args.sample_count)
    sampled_indices = list(range(0, len(points), sample_step))[: args.sample_count]

    plane_rmses: list[float] = []
    legacy_thicknesses: list[float] = []
    nearest_neighbor_distances: list[float] = []
    isolated_points = 0
    neighbor_counts: list[int] = []

    for index in sampled_indices:
        neighbors, nearest_distance = find_neighbors(
            index,
            xyz_points,
            grid,
            args.neighbor_radius,
            args.max_neighbors,
        )
        neighbor_counts.append(len(neighbors))
        if nearest_distance is not None:
            nearest_neighbor_distances.append(nearest_distance)
        if len(neighbors) < args.min_neighbors:
            isolated_points += 1
            continue

        local_cloud = [xyz_points[index], *neighbors]
        plane_rmses.append(plane_rmse(local_cloud))
        legacy_thicknesses.append(legacy_thickness(local_cloud))

    if not plane_rmses:
        raise ValueError(f"Could not compute local geometry from {csv_path}.")

    plane_rmses.sort()
    legacy_thicknesses.sort()
    nearest_neighbor_distances.sort()

    average_quality = sum(point[4] for point in points) / len(points)
    average_neighbors = sum(neighbor_counts) / len(neighbor_counts)
    isolation_ratio = isolated_points / len(sampled_indices)

    average_plane_rmse = sum(plane_rmses) / len(plane_rmses)
    p90_plane_rmse = percentile(plane_rmses, 0.9)
    median_nn_distance = percentile(nearest_neighbor_distances, 0.5)
    p90_nn_distance = percentile(nearest_neighbor_distances, 0.9)

    average_legacy_thickness = sum(legacy_thicknesses) / len(legacy_thicknesses)
    p90_legacy_thickness = percentile(legacy_thicknesses, 0.9)
    legacy_score = clamp_score(
        100.0
        - (average_legacy_thickness * 0.6)
        - (p90_legacy_thickness * 0.25)
        - (isolation_ratio * 100.0 * 0.5)
    )

    occupied_voxels = len(coarse_grid)
    supported_voxels = sum(1 for bucket in coarse_grid.values() if len(bucket) >= 2)
    supported_voxel_ratio = supported_voxels / occupied_voxels

    geometry_score = clamp_score(
        100.0
        - (average_plane_rmse * 4.0)
        - (p90_plane_rmse * 2.5)
        - (isolation_ratio * 100.0 * 0.35)
    )
    coverage_score = clamp_score(
        100.0 * math.sqrt(supported_voxels / max(args.coverage_target, 1))
    )
    support_score = clamp_score(supported_voxel_ratio * 100.0)
    composite_score = clamp_score(
        (geometry_score * 0.65)
        + (coverage_score * 0.25)
        + (support_score * 0.10)
    )

    return {
        "path": csv_path,
        "points": len(points),
        "average_quality": average_quality,
        "average_neighbors": average_neighbors,
        "isolated_ratio": isolation_ratio,
        "average_plane_rmse": average_plane_rmse,
        "p90_plane_rmse": p90_plane_rmse,
        "median_nn_distance": median_nn_distance,
        "p90_nn_distance": p90_nn_distance,
        "occupied_voxels": occupied_voxels,
        "supported_voxels": supported_voxels,
        "supported_voxel_ratio": supported_voxel_ratio,
        "legacy_score": legacy_score,
        "geometry_score": geometry_score,
        "coverage_score": coverage_score,
        "support_score": support_score,
        "composite_score": composite_score,
    }


def resolve_csv_paths(input_path: Path) -> list[Path]:
    if input_path.is_file():
        return [input_path]
    if input_path.is_dir():
        return sorted(input_path.rglob("*_points.csv"))
    raise FileNotFoundError(f"Could not find {input_path}")


def print_single_result(result: dict[str, float | int | Path], args: argparse.Namespace) -> None:
    print(f"Scan: {result['path']}")
    print(f"Points: {result['points']}")
    print(f"Average quality: {result['average_quality']:.2f}")
    print(f"Average neighbors in radius: {result['average_neighbors']:.2f}")
    print(
        f"Plane RMSE thickness: avg {result['average_plane_rmse']:.2f} mm, "
        f"p90 {result['p90_plane_rmse']:.2f} mm"
    )
    print(
        f"Nearest-neighbor distance: median {result['median_nn_distance']:.2f} mm, "
        f"p90 {result['p90_nn_distance']:.2f} mm"
    )
    print(f"Isolated sample ratio: {result['isolated_ratio']:.2%}")
    print(
        f"Supported coverage voxels ({args.coarse_voxel_size:.0f} mm): "
        f"{result['supported_voxels']} / {result['occupied_voxels']} "
        f"({result['supported_voxel_ratio']:.2%})"
    )
    print(f"Legacy score: {result['legacy_score']:.2f} / 100")
    print(f"Geometry score: {result['geometry_score']:.2f} / 100")
    print(f"Coverage score: {result['coverage_score']:.2f} / 100")
    print(f"Support score: {result['support_score']:.2f} / 100")
    print(f"Composite score: {result['composite_score']:.2f} / 100")


def print_directory_results(results: list[dict[str, float | int | Path]]) -> None:
    print("Composite\tLegacy\tGeom\tCover\tSupport\tPoints\tPath")
    for result in sorted(results, key=lambda item: item["composite_score"], reverse=True):
        print(
            f"{result['composite_score']:.2f}\t"
            f"{result['legacy_score']:.2f}\t"
            f"{result['geometry_score']:.2f}\t"
            f"{result['coverage_score']:.2f}\t"
            f"{result['support_score']:.2f}\t"
            f"{result['points']}\t"
            f"{result['path']}"
        )


def main() -> int:
    args = parse_args()
    input_path = Path(args.input_path)

    try:
        csv_paths = resolve_csv_paths(input_path)
    except FileNotFoundError as exc:
        print(exc)
        return 1

    if not csv_paths:
        print(f"No *_points.csv files found under {input_path}")
        return 1

    results = []
    for csv_path in csv_paths:
        try:
            results.append(benchmark_file(csv_path, args))
        except ValueError as exc:
            print(exc)

    if not results:
        return 1

    if len(results) == 1:
        print_single_result(results[0], args)
    else:
        print_directory_results(results)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

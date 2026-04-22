import argparse
import csv
import math
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Score a saved scan CSV for geometric quality."
    )
    parser.add_argument("input_csv", help="Path to a scan CSV file")
    parser.add_argument(
        "--neighbor-radius",
        type=float,
        default=80.0,
        help="Neighbor radius in millimeters for local flatness checks",
    )
    parser.add_argument(
        "--max-neighbors",
        type=int,
        default=24,
        help="Maximum neighbors to use per sample point",
    )
    parser.add_argument(
        "--sample-count",
        type=int,
        default=300,
        help="How many evenly spaced points to evaluate",
    )
    return parser.parse_args()


def load_points(csv_path: str) -> list[tuple[float, float, float, float, int]]:
    points: list[tuple[float, float, float, float, int]] = []
    with open(csv_path, newline="", encoding="utf-8") as handle:
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


def distance3(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    dz = a[2] - b[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def find_neighbors(
    index: int,
    xyz_points: list[tuple[float, float, float]],
    radius: float,
    max_neighbors: int,
) -> list[tuple[float, float, float]]:
    center = xyz_points[index]
    candidates: list[tuple[float, tuple[float, float, float]]] = []
    for neighbor_index, point in enumerate(xyz_points):
        if neighbor_index == index:
            continue
        dist = distance3(center, point)
        if dist <= radius:
            candidates.append((dist, point))

    candidates.sort(key=lambda item: item[0])
    return [point for _, point in candidates[:max_neighbors]]


def axis_spread(values: list[float]) -> float:
    return max(values) - min(values)


def local_plane_thickness(points: list[tuple[float, float, float]]) -> float:
    spreads = [
        axis_spread([point[0] for point in points]),
        axis_spread([point[1] for point in points]),
        axis_spread([point[2] for point in points]),
    ]
    spreads.sort()
    return spreads[0]


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


def main() -> int:
    args = parse_args()
    csv_path = Path(args.input_csv)
    points = load_points(args.input_csv)
    if len(points) < 20:
        print(f"Not enough points in {csv_path} to evaluate quality.")
        return 1

    xyz_points = [(point[0], point[1], point[2]) for point in points]
    step = max(1, len(points) // args.sample_count)
    sampled_indices = list(range(0, len(points), step))[: args.sample_count]

    local_thicknesses: list[float] = []
    isolated_points = 0
    local_neighbor_counts: list[int] = []
    quality_values = [point[4] for point in points]

    for index in sampled_indices:
        neighbors = find_neighbors(index, xyz_points, args.neighbor_radius, args.max_neighbors)
        local_neighbor_counts.append(len(neighbors))
        if len(neighbors) < 6:
            isolated_points += 1
            continue

        local_cloud = [xyz_points[index], *neighbors]
        local_thicknesses.append(local_plane_thickness(local_cloud))

    if not local_thicknesses:
        print(f"Could not compute local flatness from {csv_path}.")
        return 1

    sorted_thicknesses = sorted(local_thicknesses)
    average_thickness = sum(local_thicknesses) / len(local_thicknesses)
    p90_thickness = percentile(sorted_thicknesses, 0.9)
    isolation_ratio = isolated_points / len(sampled_indices)
    average_neighbors = sum(local_neighbor_counts) / len(local_neighbor_counts)
    average_quality = sum(quality_values) / len(quality_values)

    # Higher is better. This favors thin local surfaces and fewer isolated points.
    score = max(0.0, 100.0 - (average_thickness * 0.6) - (p90_thickness * 0.25) - (isolation_ratio * 100.0 * 0.5))

    print(f"Scan: {csv_path}")
    print(f"Points: {len(points)}")
    print(f"Average quality: {average_quality:.2f}")
    print(f"Average neighbors in radius: {average_neighbors:.2f}")
    print(f"Average local thickness: {average_thickness:.2f} mm")
    print(f"90th percentile local thickness: {p90_thickness:.2f} mm")
    print(f"Isolated sample ratio: {isolation_ratio:.2%}")
    print(f"Quality score: {score:.2f} / 100")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

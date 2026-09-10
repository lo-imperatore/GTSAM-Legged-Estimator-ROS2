#!/usr/bin/env python3
"""Evaluate vertical relative-pose error from ROS 2 bags.

Pose pairs are selected from distance travelled along the reference trajectory.
Each estimator is associated to the reference using message header timestamps.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import matplotlib.pyplot as plt
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore


@dataclass(frozen=True)
class Trajectory:
    timestamps: np.ndarray
    positions: np.ndarray


def _header_time(message: object, fallback_ns: int) -> float:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None or not (stamp.sec or stamp.nanosec):
        return fallback_ns * 1.0e-9
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _position(message: object) -> np.ndarray:
    pose = getattr(message, "pose")
    if hasattr(pose, "pose"):
        pose = pose.pose
    position = pose.position
    return np.array([position.x, position.y, position.z], dtype=float)


def _load_trajectories(
    bag: Path, topics: Iterable[str]
) -> dict[str, Trajectory]:
    requested = set(topics)
    samples: dict[str, list[tuple[float, np.ndarray]]] = {
        topic: [] for topic in requested
    }
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    with AnyReader([bag], default_typestore=typestore) as reader:
        connections = [
            connection
            for connection in reader.connections
            if connection.topic in requested
        ]
        found = {connection.topic for connection in connections}
        missing = sorted(requested - found)
        if missing:
            raise RuntimeError(
                "Topics not found in bag: " + ", ".join(missing)
            )
        for connection, bag_timestamp, rawdata in reader.messages(
            connections=connections
        ):
            message = reader.deserialize(rawdata, connection.msgtype)
            samples[connection.topic].append(
                (_header_time(message, bag_timestamp), _position(message))
            )

    trajectories: dict[str, Trajectory] = {}
    for topic, values in samples.items():
        if len(values) < 2:
            raise RuntimeError(f"Topic {topic!r} contains fewer than 2 poses")
        values.sort(key=lambda value: value[0])
        timestamps = np.asarray([value[0] for value in values])
        positions = np.asarray([value[1] for value in values])
        unique = np.concatenate(([True], np.diff(timestamps) > 0.0))
        trajectories[topic] = Trajectory(
            timestamps=timestamps[unique], positions=positions[unique]
        )
    return trajectories


def _associate(
    reference: Trajectory,
    estimate: Trajectory,
    max_time_difference: float,
) -> tuple[np.ndarray, np.ndarray]:
    right = np.searchsorted(estimate.timestamps, reference.timestamps)
    right = np.clip(right, 0, len(estimate.timestamps) - 1)
    left = np.clip(right - 1, 0, len(estimate.timestamps) - 1)
    choose_left = (
        np.abs(estimate.timestamps[left] - reference.timestamps)
        <= np.abs(estimate.timestamps[right] - reference.timestamps)
    )
    indices = np.where(choose_left, left, right)
    differences = np.abs(
        estimate.timestamps[indices] - reference.timestamps
    )
    valid = differences <= max_time_difference
    return indices, valid


def _reference_pairs(
    positions: np.ndarray,
    delta_m: float,
    tolerance: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    segment_lengths = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    distance = np.concatenate(([0.0], np.cumsum(segment_lengths)))
    starts: list[int] = []
    ends: list[int] = []
    pair_distances: list[float] = []
    maximum_error = delta_m * tolerance
    for start in range(len(distance) - 1):
        end = int(np.searchsorted(distance, distance[start] + delta_m))
        if end >= len(distance):
            break
        travelled = distance[end] - distance[start]
        if abs(travelled - delta_m) <= maximum_error:
            starts.append(start)
            ends.append(end)
            pair_distances.append(distance[start])
    if not starts:
        raise RuntimeError(
            "No reference pose pairs matched the requested distance; "
            "increase --delta-tolerance"
        )
    return (
        np.asarray(starts, dtype=int),
        np.asarray(ends, dtype=int),
        np.asarray(pair_distances),
    )


def _statistics(errors: np.ndarray) -> dict[str, float]:
    absolute = np.abs(errors)
    return {
        "count": float(len(errors)),
        "rmse": float(np.sqrt(np.mean(np.square(errors)))),
        "mean_abs": float(np.mean(absolute)),
        "median_abs": float(np.median(absolute)),
        "std": float(np.std(errors)),
        "min_abs": float(np.min(absolute)),
        "max_abs": float(np.max(absolute)),
    }


def _parse_estimator(value: str) -> tuple[str, str]:
    if "=" not in value:
        raise argparse.ArgumentTypeError(
            "Estimator must use LABEL=TOPIC syntax"
        )
    label, topic = value.split("=", 1)
    if not label or not topic:
        raise argparse.ArgumentTypeError(
            "Estimator label and topic must both be non-empty"
        )
    return label, topic


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--reference-topic", required=True)
    parser.add_argument(
        "--estimator",
        action="append",
        type=_parse_estimator,
        required=True,
        metavar="LABEL=TOPIC",
    )
    parser.add_argument("--delta", type=float, default=1.0)
    parser.add_argument("--delta-tolerance", type=float, default=0.1)
    parser.add_argument("--max-time-diff", type=float, default=0.02)
    parser.add_argument(
        "--output-dir", type=Path, default=Path("z_rpe_results")
    )
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args()

    if args.delta <= 0.0:
        parser.error("--delta must be positive")
    if not 0.0 <= args.delta_tolerance <= 1.0:
        parser.error("--delta-tolerance must be between 0 and 1")
    if args.max_time_diff < 0.0:
        parser.error("--max-time-diff must be non-negative")

    topics = [args.reference_topic] + [topic for _, topic in args.estimator]
    trajectories = _load_trajectories(args.bag, topics)
    reference = trajectories[args.reference_topic]
    starts, ends, pair_distances = _reference_pairs(
        reference.positions, args.delta, args.delta_tolerance
    )

    args.output_dir.mkdir(parents=True, exist_ok=True)
    summary_rows: list[dict[str, object]] = []
    plot_rows: list[tuple[str, np.ndarray, np.ndarray]] = []

    for label, topic in args.estimator:
        associated, valid = _associate(
            reference, trajectories[topic], args.max_time_diff
        )
        valid_pairs = valid[starts] & valid[ends]
        pair_starts = starts[valid_pairs]
        pair_ends = ends[valid_pairs]
        estimate_starts = associated[pair_starts]
        estimate_ends = associated[pair_ends]
        estimate_z = trajectories[topic].positions[:, 2]
        reference_z = reference.positions[:, 2]
        errors = (
            estimate_z[estimate_ends]
            - estimate_z[estimate_starts]
            - reference_z[pair_ends]
            + reference_z[pair_starts]
        )
        if not len(errors):
            raise RuntimeError(
                f"No synchronized pose pairs remained for {label!r}"
            )

        distances = pair_distances[valid_pairs]
        csv_path = args.output_dir / f"{label}_z_rpe.csv"
        with csv_path.open("w", newline="", encoding="utf-8") as output:
            writer = csv.writer(output)
            writer.writerow(
                ["reference_distance_m", "reference_time_s", "z_rpe_m"]
            )
            for distance, start, error in zip(
                distances, pair_starts, errors
            ):
                writer.writerow(
                    [distance, reference.timestamps[start], error]
                )

        stats = _statistics(errors)
        summary_rows.append({"label": label, **stats})
        plot_rows.append((label, distances, errors))

    summary_path = args.output_dir / "z_rpe_summary.csv"
    with summary_path.open("w", newline="", encoding="utf-8") as output:
        fieldnames = [
            "label", "count", "rmse", "mean_abs", "median_abs", "std",
            "min_abs", "max_abs",
        ]
        writer = csv.DictWriter(output, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(summary_rows)

    print(f"Z-RPE over {args.delta:g} m reference intervals")
    for row in summary_rows:
        print(
            f"  {row['label']}: count={int(row['count'])}, "
            f"rmse={row['rmse']:.6f} m, "
            f"mean_abs={row['mean_abs']:.6f} m, "
            f"max_abs={row['max_abs']:.6f} m"
        )
    print(f"Results: {args.output_dir}")

    if not args.no_plot:
        figure, axis = plt.subplots(figsize=(12, 5))
        for label, distances, errors in plot_rows:
            axis.plot(distances, errors, label=label, linewidth=0.8)
        axis.axhline(0.0, color="black", linewidth=0.7)
        axis.set_xlabel("Reference distance [m]")
        axis.set_ylabel("Z relative-pose error [m]")
        axis.grid(True, alpha=0.3)
        axis.legend()
        figure.tight_layout()
        plot_path = args.output_dir / "z_rpe.png"
        figure.savefig(plot_path, dpi=160)
        print(f"Plot: {plot_path}")


if __name__ == "__main__":
    main()

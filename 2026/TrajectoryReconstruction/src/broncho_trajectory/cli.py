"""命令行入口。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .centerline import join_routes, load_vtp_lines, map_progress_to_route, similarity_register
from .geometry import distance
from .io import load_record, write_trajectory_csv, write_trajectory_json
from .kinematics import KinematicConfig, reconstruct
from .visualization import registered_html, trajectory_html


def _config(args: argparse.Namespace) -> KinematicConfig:
    return KinematicConfig(
        m0_mm_per_degree=args.m0_mm_per_degree,
        m1_roll_deg_per_degree=args.m1_roll_deg_per_degree,
        m2_tip_deg_per_degree=args.m2_tip_deg_per_degree,
        m0_sign=args.m0_sign,
        m1_sign=args.m1_sign,
        m2_sign=args.m2_sign,
        m1_zero_degree=args.m1_zero_degree,
    )


def _add_calibration(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--m0-mm-per-degree", type=float, default=0.10)
    parser.add_argument("--m1-roll-deg-per-degree", type=float, default=1.0)
    parser.add_argument("--m2-tip-deg-per-degree", type=float, default=1.0)
    parser.add_argument("--m0-sign", type=float, default=-1.0)
    parser.add_argument("--m1-sign", type=float, default=1.0)
    parser.add_argument("--m2-sign", type=float, default=1.0)
    parser.add_argument("--m1-zero-degree", type=float, default=0.0)


def command_reconstruct(args: argparse.Namespace) -> None:
    record = load_record(args.input_json)
    points = reconstruct(record["frames"], _config(args))
    output_dir = args.output_dir or args.input_json.parent
    output_dir.mkdir(parents=True, exist_ok=True)
    stem = args.input_json.stem
    metadata = {
        "source": args.input_json.name,
        "session_name": record.get("session_name"),
        "path_name": record.get("path_name"),
        "config": vars(_config(args)),
        "warning": "仅编码器运动学估计，不等同于真实空间定位。",
    }
    json_path = output_dir / f"{stem}_trajectory.json"
    csv_path = output_dir / f"{stem}_trajectory.csv"
    html_path = output_dir / f"{stem}_trajectory.html"
    write_trajectory_json(json_path, points, metadata)
    write_trajectory_csv(csv_path, points)
    html_path.write_text(
        trajectory_html(points, f"支气管镜三维轨迹 — {record.get('path_name', stem)}"),
        encoding="utf-8",
    )
    print(f"生成 {len(points)} 个轨迹点")
    print(json_path)
    print(csv_path)
    print(html_path)


def command_register(args: argparse.Namespace) -> None:
    record = load_record(args.input_json)
    points = reconstruct(record["frames"], _config(args))
    route = join_routes(args.route)
    progress = [point.insertion_mm for point in points]
    corrected = map_progress_to_route(progress, route)
    raw = [point.position for point in points]
    raw_registered = similarity_register(raw, corrected)
    errors = [distance(a, b) for a, b in zip(raw_registered, corrected)]
    model_lines = load_vtp_lines(args.model_vtp) if args.model_vtp else []
    output = args.output or args.input_json.with_name(args.input_json.stem + "_registered.html")
    output.write_text(
        registered_html(
            f"{record.get('path_name', args.input_json.stem)}：轨迹与 CT 中心线",
            model_lines,
            route,
            corrected,
            raw_registered,
            [point.timestamp for point in points],
            errors,
        ),
        encoding="utf-8",
    )
    metrics_path = output.with_suffix(".metrics.json")
    metrics_path.write_text(
        json.dumps(
            {
                "frames": len(points),
                "mean_error_mm": sum(errors) / len(errors),
                "max_error_mm": max(errors),
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )
    print(output)
    print(metrics_path)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="broncho-trajectory", description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    reconstruct_parser = subparsers.add_parser("reconstruct", help="由 M0/M1/M2 重建三维轨迹")
    reconstruct_parser.add_argument("input_json", type=Path)
    reconstruct_parser.add_argument("--output-dir", type=Path)
    _add_calibration(reconstruct_parser)
    reconstruct_parser.set_defaults(handler=command_reconstruct)

    register_parser = subparsers.add_parser("register", help="将轨迹配准并叠加到 CT 中心线")
    register_parser.add_argument("input_json", type=Path)
    register_parser.add_argument("--route", type=Path, action="append", required=True,
                                 help="有序的 Slicer .mrk.json 路径，可重复提供")
    register_parser.add_argument("--model-vtp", type=Path, help="可选的完整中心线 VTP")
    register_parser.add_argument("--output", type=Path)
    _add_calibration(register_parser)
    register_parser.set_defaults(handler=command_register)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    args.handler(args)


if __name__ == "__main__":
    main()

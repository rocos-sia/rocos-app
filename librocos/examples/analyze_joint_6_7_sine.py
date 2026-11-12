#!/usr/bin/env python3
"""分析 joint_6_7_sine 的 CSV，输出 JSON、Markdown 和曲线。"""

import argparse
import json
from pathlib import Path

import numpy as np


def amplitudes(metadata):
    return metadata.get("joint_amplitudes_rad", [0] * 5 + [metadata["amplitude_rad"]] * 2)


def error_stats(values):
    return {
        "rms_rad": float(np.sqrt(np.mean(values**2))),
        "p99_rad": float(np.quantile(np.abs(values), .99)),
        "peak_rad": float(np.max(np.abs(values))),
    }


def phase_difference(first, second):
    return float((first - second + np.pi) % (2 * np.pi) - np.pi)


def analyze(data, metadata, outcome):
    if len(data) < 2:
        raise ValueError("Need at least two complete CSV samples")
    time = data["time_s"]
    if np.any(np.diff(time) <= 0):
        raise ValueError("Non-increasing sample time")
    for field in data.dtype.names:
        if not np.isfinite(data[field]).all():
            raise ValueError(f"Nonfinite CSV field {field}")
    duration = metadata["duration_s"]
    ramp = metadata["ramp_s"]
    omega = 2 * np.pi * metadata["frequency_hz"]
    steady = (time >= ramp + 1) & (time <= duration - ramp - 1)
    quality = data["quality"][data["applied_id"] > 100]
    cycles = data["cycle"].astype(np.int64)
    if np.any(np.diff(cycles) <= 0):
        raise ValueError("Non-increasing control cycle")
    consecutive = np.diff(cycles) == 1
    report = {
        "samples": len(data), "observed_duration_s": float(time[-1] - time[0]),
        "completed": bool(outcome.get("completed", False)), "outcome": outcome,
        "trajectory_time_basis": metadata.get("trajectory_time_basis", "sample_time"),
        "actual_observed_duration_s": float((data["sample_ns"][-1] - data["sample_ns"][0]) * 1e-9),
        "sample_interval_ms": dict(zip(
            ("p50", "p99", "max"), map(float, np.quantile(np.diff(data["sample_ns"]) * 1e-6, [.5, .99, 1])))),
        "minimum_command_success_rate": float(quality.min()) if len(quality) else None,
        "mean_command_success_rate": float(quality.mean()) if len(quality) else None,
        "observed_cycle_gaps": int(np.count_nonzero(np.diff(cycles) > 1)),
        "consecutive_cycle_pairs": int(consecutive.sum()),
        "reference_jerk_note": "Peak jerk uses only consecutive observed control cycles and configured dt",
        "requested_note": "requested is generated from this callback's sample time for the next transmission; applied_id does not map it to an applied target",
        "joints": {},
    }
    for joint, amplitude in enumerate(amplitudes(metadata), 1):
        if amplitude == 0:
            continue
        requested, q, qd, command = (data[f"{field}{joint}"] for field in ("requested", "q", "qd", "qcmd"))
        limits = metadata["limits"][joint - 1]
        observed_jerk = np.diff(data[f"ddqd{joint}"])[consecutive] / metadata["cycle_s"]
        row = {
            "requested_to_actual": error_stats(q - requested),
            "requested_to_reference": error_stats(qd - requested),
            "output_to_actual": error_stats(q - command),
            "reference_to_output": error_stats(command - qd),
            "initial_rad": float(q[0]), "final_rad": float(q[-1]),
            "endpoint_change_rad": float(q[-1] - q[0]),
            "actual_range_rad": [float(q.min()), float(q.max())],
            "actual_speed_peak_rad_s": float(np.max(np.abs(data[f"dq{joint}"]))),
            "reference_speed_peak_rad_s": float(np.max(np.abs(data[f"dqd{joint}"]))),
            "reference_acceleration_peak_rad_s2": float(np.max(np.abs(data[f"ddqd{joint}"]))),
            "observed_reference_jerk_peak_rad_s3": float(np.max(np.abs(observed_jerk))) if len(observed_jerk) else None,
            "reference_acceleration_limit_fraction": float(np.mean(np.abs(data[f"ddqd{joint}"]) >= limits["acceleration"] * (1 - 1e-6))),
            "observed_reference_jerk_limit_fraction": float(np.mean(np.abs(observed_jerk) >= limits["jerk"] * (1 - 1e-6))) if len(observed_jerk) else None,
            "position_violation_samples": int(np.count_nonzero((q < limits["lower"]) | (q > limits["upper"]))),
            "following_violation_samples": int(np.count_nonzero(np.abs(q - command) > limits["following_error"])),
            "steady_fit": None,
        }
        if steady.sum() >= 100 and np.ptp(time[steady]) >= 2 / metadata["frequency_hz"]:
            basis = np.column_stack((np.sin(omega * time[steady]), np.cos(omega * time[steady]), np.ones(steady.sum())))
            fits = {}
            for label, signal in (("requested", requested), ("reference", qd), ("output", command), ("actual", q)):
                coefficients = np.linalg.lstsq(basis, signal[steady], rcond=None)[0]
                fits[label] = {
                    "amplitude_rad": float(np.hypot(*coefficients[:2])),
                    "phase_rad": float(np.arctan2(coefficients[1], coefficients[0])),
                    "offset_rad": float(coefficients[2]),
                    "residual_rms_rad": float(np.sqrt(np.mean((signal[steady] - basis @ coefficients)**2))),
                }
            row["steady_fit"] = {
                "signals": fits,
                "amplitude_ratio": fits["actual"]["amplitude_rad"] / fits["requested"]["amplitude_rad"],
                "total_delay_ms": -phase_difference(fits["actual"]["phase_rad"], fits["requested"]["phase_rad"]) / omega * 1000,
                "processing_delay_ms": -phase_difference(fits["reference"]["phase_rad"], fits["requested"]["phase_rad"]) / omega * 1000,
                "following_delay_ms": -phase_difference(fits["actual"]["phase_rad"], fits["output"]["phase_rad"]) / omega * 1000,
            }
        report["joints"][f"J{joint}"] = row
    report["held_joint_peak_displacement_rad"] = {
        f"J{joint}": float(np.max(np.abs(data[f"q{joint}"] - data[f"q{joint}"][0])))
        for joint, amplitude in enumerate(amplitudes(metadata), 1) if amplitude == 0
    }
    return report


def write_report(directory, report, metadata):
    joint_text = "、".join(f"J{i} ±{np.degrees(a):.1f}°" for i, a in enumerate(amplitudes(metadata), 1) if a > 0)
    lines = ["# 关节正弦伺服数据报告", "",
             f"数据来源为 {metadata.get('data_source', '未标注')}。",
             f"{joint_text}，频率 {metadata['frequency_hz']:.3f} Hz，零位中心，运动轴同相。",
             f"轨迹时间基准为 {report['trajectory_time_basis']}，实际记录时长 {report['actual_observed_duration_s']:.6f} s。",
             f"计划运动 {metadata['duration_s']:.3f} s，记录跨度 {report['observed_duration_s']:.3f} s，采样 {report['samples']} 条。",
             f"程序完成标志为 `{report['completed']}`，退出状态详见 `outcome.json`。", "",
             "| 关节 | 总误差 RMS mrad | 总误差峰值 mrad | 参考处理 RMS mrad | 实际跟随 RMS mrad | 最后位置 rad |",
             "|---|---:|---:|---:|---:|---:|"]
    for joint, row in report["joints"].items():
        lines.append(f"| {joint} | {row['requested_to_actual']['rms_rad']*1000:.4f} | "
                     f"{row['requested_to_actual']['peak_rad']*1000:.4f} | "
                     f"{row['requested_to_reference']['rms_rad']*1000:.4f} | "
                     f"{row['output_to_actual']['rms_rad']*1000:.4f} | {row['final_rad']:.9f} |")
    lines += ["", "## 稳态幅相", "",
              "| 关节 | 实际幅值/请求幅值 | 总等效滞后 ms | 参考处理等效滞后 ms | 输出到反馈等效滞后 ms |",
              "|---|---:|---:|---:|---:|"]
    for joint, row in report["joints"].items():
        fit = row["steady_fit"]
        if fit:
            lines.append(f"| {joint} | {fit['amplitude_ratio']:.6f} | {fit['total_delay_ms']:.3f} | "
                         f"{fit['processing_delay_ms']:.3f} | {fit['following_delay_ms']:.3f} |")
        else:
            lines.append(f"| {joint} | 稳态数据不足 | — | — | — |")
    lines += ["", "## 通信与采样", "",
              f"最低新指令接收比例为 `{report['minimum_command_success_rate']}`。",
              f"采样间隔 P99 为 {report['sample_interval_ms']['p99']:.3f} ms，峰值为 {report['sample_interval_ms']['max']:.3f} ms。",
              f"观察到控制周期跳号 {report['observed_cycle_gaps']} 次，连续周期对 {report['consecutive_cycle_pairs']} 个。", "",
              "## 统计口径", "",
              "`requested` 是本次回调根据 metadata 中的轨迹时间基准生成、供随后发送的目标。与它比较的误差包含客户端提交时序。",
              "采样间隔由 sample_ns 实际时间差计算，time_s 在 control_cycle 模式下表示名义轨迹时间。",
              "`q`、`q_d` 和 `q_cmd` 来自同一服务端周期快照，输出到反馈的误差使用这些字段直接计算。",
              "`applied_id` 保留供诊断，CSV 没有每个发包序号到原始目标的映射。",
              "幅相拟合采用稳态区间的正弦、余弦和常数项，滞后为当前频率下的等效值。",
              "周期跳号包含客户端未观察到中间状态的情况。jerk 仅统计连续采样到的控制周期，完整峰值需要控制器 trace。",
              "反馈源时间与采样时间可能采用不同的时钟基准，程序分别保留。",
              "关节边界、跟随误差、导数触限及保持轴位移统计见 `analysis.json`。",
              "位置跟随误差超阈值计数用于诊断，不代表控制器因此停止；保护行为需结合客户端与服务端版本。", "",
              "![轨迹、误差与通信采样](analysis.png)", ""]
    (directory / "report.md").write_text("\n".join(lines), encoding="utf-8")


def plot(directory, data, metadata):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    joints = [i for i, a in enumerate(amplitudes(metadata), 1) if a > 0]
    fig, axes = plt.subplots(len(joints) + 1, 2, figsize=(13, 3 * (len(joints) + 1)), constrained_layout=True)
    fig.suptitle(metadata.get("data_source", "Recording source unspecified"), fontsize=11)
    step = max(1, len(data) // 8000)
    s = data[::step]
    for row, joint in enumerate(joints):
        for field, label in (("requested", "Requested"), ("qd", "Reference"), ("q", "Actual")):
            axes[row, 0].plot(s["time_s"], np.degrees(s[f"{field}{joint}"]), label=label, linewidth=1)
        axes[row, 0].set(title=f"J{joint} joint position", ylabel="deg")
        axes[row, 1].plot(s["time_s"], (s[f"q{joint}"]-s[f"requested{joint}"])*1000, label="Total")
        axes[row, 1].plot(s["time_s"], (s[f"q{joint}"]-s[f"qcmd{joint}"])*1000, label="Output to actual")
        axes[row, 1].set(title=f"J{joint} tracking error", ylabel="mrad")
    axes[-1, 0].plot(s["time_s"], s["quality"], label="New command ratio")
    axes[-1, 0].set(title="Communication quality", ylim=(0, 1.05))
    axes[-1, 1].plot(s["time_s"], s["dt_s"]*1000, label="Observed interval")
    axes[-1, 1].set(title="Callback sampling interval", ylabel="ms")
    for ax in axes.flat:
        ax.set_xlabel("Time (s)")
        ax.grid(alpha=.2)
        ax.legend()
    fig.savefig(directory / "analysis.png", dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    metadata = json.loads((args.directory / "metadata.json").read_text())
    outcome_path = args.directory / "outcome.json"
    outcome = json.loads(outcome_path.read_text()) if outcome_path.exists() else {
        "completed": False, "error": "Missing terminal outcome, recording may be incomplete"}
    data = np.atleast_1d(np.genfromtxt(args.directory / "samples.csv", delimiter=",", names=True, dtype=None, encoding="utf-8"))
    report = analyze(data, metadata, outcome)
    (args.directory / "analysis.json").write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    write_report(args.directory, report, metadata)
    plot(args.directory, data, metadata)
    print(args.directory / "report.md")


if __name__ == "__main__":
    main()

from __future__ import annotations

import csv
from pathlib import Path
from typing import Dict, List

from config import LEGS

ROOT = Path(__file__).resolve().parent
RESULTS = ROOT / "results"


def read_rows(path: Path) -> List[Dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def main() -> None:
    telemetry = RESULTS / "telemetry.csv"
    if not telemetry.exists():
        raise SystemExit("results/telemetry.csv not found. Run simulate.py first.")

    rows = read_rows(telemetry)
    if not rows:
        raise SystemExit("Telemetry file is empty.")

    out = RESULTS / "fea_load_cases.csv"
    fields = [
        "leg",
        "event",
        "time_s",
        "motor_torque_Nm",
        "foot_normal_N",
        "crank_force_N",
        "crank_torque_Nm",
        "coupler_force_N",
        "coupler_torque_Nm",
        "rocker_force_N",
        "rocker_torque_Nm",
        "ankle_force_N",
        "ankle_torque_Nm",
        "body_roll_deg",
        "body_pitch_deg",
    ]

    cases = []
    for leg in LEGS:
        metrics = {
            "peak_foot_force": f"{leg}_foot_normal_N",
            "peak_crank_force": f"{leg}_crank_force_N",
            "peak_coupler_force": f"{leg}_coupler_force_N",
            "peak_rocker_force": f"{leg}_rocker_force_N",
            "peak_ankle_force": f"{leg}_ankle_force_N",
            "peak_motor_torque": f"{leg}_motor_torque_Nm",
        }
        for event, field in metrics.items():
            row = max(rows, key=lambda r: abs(float(r.get(field, 0.0))))
            cases.append({
                "leg": leg,
                "event": event,
                "time_s": row["time_s"],
                "motor_torque_Nm": row[f"{leg}_motor_torque_Nm"],
                "foot_normal_N": row[f"{leg}_foot_normal_N"],
                "crank_force_N": row[f"{leg}_crank_force_N"],
                "crank_torque_Nm": row[f"{leg}_crank_torque_Nm"],
                "coupler_force_N": row[f"{leg}_coupler_force_N"],
                "coupler_torque_Nm": row[f"{leg}_coupler_torque_Nm"],
                "rocker_force_N": row[f"{leg}_rocker_force_N"],
                "rocker_torque_Nm": row[f"{leg}_rocker_torque_Nm"],
                "ankle_force_N": row[f"{leg}_ankle_force_N"],
                "ankle_torque_Nm": row[f"{leg}_ankle_torque_Nm"],
                "body_roll_deg": row["roll_deg"],
                "body_pitch_deg": row["pitch_deg"],
            })

    with out.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(cases)

    print(f"Wrote FEA-oriented load cases: {out}")
    print("Use these reaction loads as boundary-condition inputs in Fusion/ANSYS/CalculiX.")


if __name__ == "__main__":
    main()

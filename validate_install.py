from __future__ import annotations

import sys
from pathlib import Path

print("Python:", sys.version)

try:
    import mujoco
    print("MuJoCo:", mujoco.__version__)
except Exception as exc:
    raise SystemExit(f"MuJoCo import failed: {exc}")

from generate_model import DEFAULT_MODEL_PATH, write_model

write_model(DEFAULT_MODEL_PATH)
model = mujoco.MjModel.from_xml_path(str(DEFAULT_MODEL_PATH))
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)

print(f"Model loaded: nbody={model.nbody}, njnt={model.njnt}, nu={model.nu}, neq={model.neq}")
print(f"Total mass: {model.body_mass.sum():.3f} kg")
print("Validation passed.")

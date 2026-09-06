from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def run(cmd):
    print("+", " ".join(str(x) for x in cmd))
    subprocess.run([str(x) for x in cmd], check=True)


def main():
    ap = argparse.ArgumentParser(description="One-command V3-F mechanical iteration loop")
    ap.add_argument("--generated", type=Path, default=ROOT / "cad" / "generated")
    ap.add_argument("--skip-cad", action="store_true", help="Reuse existing generated STEP files")
    ap.add_argument("--skip-render", action="store_true", help="Skip endpoint CAD renders")
    ap.add_argument("--skip-motion", action="store_true", help="Skip the two short CAD-derived motion clips")
    args = ap.parse_args()

    if not args.skip_cad:
        run([sys.executable, ROOT / "cad" / "generate_v3f.py"])
    run([sys.executable, ROOT / "tools" / "validate_v3f.py"])
    if not args.skip_render:
        run([sys.executable, ROOT / "tools" / "render_step.py", "--generated", args.generated, "--out", ROOT / "output" / "renders"])
    if not args.skip_motion:
        run([sys.executable, ROOT / "tools" / "render_motion.py"])
    print("\nV3-F iteration complete. Review output/V3F_STATUS.md, output/renders/, and output/motion/.")


if __name__ == "__main__":
    main()

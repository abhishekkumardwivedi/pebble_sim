from __future__ import annotations

"""Render short CAD-derived expression clips without FreeCAD or MuJoCo.

Every frame is regenerated from the parametric V3-F solids at interpolated motor
angles. The output is therefore a visualization of the CAD model, not an AI
image or a 2-D cartoon approximation.
"""

import argparse
import importlib.util
import json
import sys
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
import vtk

ROOT = Path(__file__).resolve().parents[1]
TOOLS = Path(__file__).resolve().parent
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

import validate_v3f as validate  # noqa: E402


def load_generator():
    path = ROOT / "cad" / "generate_v3f.py"
    spec = importlib.util.spec_from_file_location("pebble_v3f_generator", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load {path}")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def polydata(shape, lin_tol=0.8, ang_tol=0.18):
    verts, tris = shape.tessellate(lin_tol, ang_tol)
    pts = vtk.vtkPoints()
    pts.SetNumberOfPoints(len(verts))
    for i, v in enumerate(verts):
        pts.SetPoint(i, v.x, v.y, v.z)
    cells = vtk.vtkCellArray()
    for tri in tris:
        c = vtk.vtkTriangle()
        for j, idx in enumerate(tri):
            c.GetPointIds().SetId(j, int(idx))
        cells.InsertNextCell(c)
    result = vtk.vtkPolyData()
    result.SetPoints(pts)
    result.SetPolys(cells)
    return result


def add_grid(renderer, extent=110.0, spacing=20.0):
    points = vtk.vtkPoints()
    lines = vtk.vtkCellArray()
    idx = 0
    vals = np.arange(-extent, extent + 0.1, spacing)
    for v in vals:
        for a, b in (((-extent, v, 0.0), (extent, v, 0.0)), ((v, -extent, 0.0), (v, extent, 0.0))):
            points.InsertNextPoint(*a)
            points.InsertNextPoint(*b)
            line = vtk.vtkLine()
            line.GetPointIds().SetId(0, idx)
            line.GetPointIds().SetId(1, idx + 1)
            idx += 2
            lines.InsertNextCell(line)
    pd = vtk.vtkPolyData()
    pd.SetPoints(points)
    pd.SetLines(lines)
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(pd)
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(0.84, 0.84, 0.82)
    actor.GetProperty().SetLineWidth(0.6)
    renderer.AddActor(actor)


def render_assembly(asm, out: Path, metrics: dict, size=(700, 620)):
    renderer = vtk.vtkRenderer()
    renderer.SetBackground(0.975, 0.975, 0.97)
    add_grid(renderer)
    dz = float(metrics["belly_clearance_mm"]) - 8.0
    roll = float(metrics["roll_deg"])
    pitch = float(metrics["pitch_deg"])

    for _, item in asm.objects.items():
        if item.obj is None:
            continue
        obj = item.obj
        shape = obj.val() if hasattr(obj, "val") else obj
        if shape is None:
            continue
        try:
            pd = polydata(shape)
        except Exception:
            continue
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(pd)
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        rgba = item.color.toTuple() if item.color is not None else (0.7, 0.7, 0.7, 1.0)
        actor.GetProperty().SetColor(*rgba[:3])
        actor.GetProperty().SetOpacity(float(rgba[3]))
        actor.GetProperty().SetInterpolationToPhong()
        # The CAD is body-fixed. This world transform shows the body attitude
        # required for the four ankle endpoints to share the floor plane.
        actor.RotateX(roll)
        actor.RotateY(pitch)
        actor.SetPosition(0.0, 0.0, dz)
        renderer.AddActor(actor)

    win = vtk.vtkRenderWindow()
    win.SetOffScreenRendering(1)
    win.AddRenderer(renderer)
    win.SetSize(*size)
    cam = renderer.GetActiveCamera()
    cam.SetPosition(270, -250, 205)
    cam.SetFocalPoint(0, 0, 74)
    cam.SetViewUp(0, 0, 1)
    renderer.ResetCameraClippingRange()
    win.Render()
    grab = vtk.vtkWindowToImageFilter()
    grab.SetInput(win)
    grab.Update()
    writer = vtk.vtkPNGWriter()
    writer.SetFileName(str(out))
    writer.SetInputConnection(grab.GetOutputPort())
    writer.Write()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--transition", action="append", help="Transition name; repeat for multiple clips")
    ap.add_argument("--out", type=Path, default=ROOT / "output" / "motion")
    ap.add_argument("--frames", type=int, default=None)
    args = ap.parse_args()

    iteration = json.loads((ROOT / "shared" / "iteration_config.json").read_text())
    mech = json.loads((ROOT / "shared" / "v3f_mechanical_spec.json").read_text())
    poses = mech["pose_targets"]
    transitions = {t["name"]: t for t in iteration["transitions"]}
    selected = args.transition or iteration.get("motion_preview_transitions", [])
    frames = max(3, int(args.frames or iteration.get("motion_review_frames", 7)))
    args.out.mkdir(parents=True, exist_ok=True)

    gen = load_generator()
    sleepy_pts = validate.pose_points(mech, poses["sleepy_compact"])
    sleepy_clearance = float(iteration["validation_targets"]["nominal_sleepy_belly_clearance_mm"])

    for name in selected:
        t = transitions[name]
        a, b = poses[t["from"]], poses[t["to"]]
        frame_paths = []
        for i, u in enumerate(np.linspace(0.0, 1.0, frames)):
            uu = max(0.0, min(1.0, float(u)))
            q = uu * uu * uu * (uu * (uu * 6.0 - 15.0) + 10.0)
            pose = {
                k: float(a[k]) + (float(b[k]) - float(a[k])) * q
                for k in ("FL", "FR", "RL", "RR", "front_splay", "rear_splay")
            }
            frame_key = "__motion_frame__"
            gen.POSES[frame_key] = pose
            asm = gen.build_pose(frame_key, include_shell=True, cutaway=False)
            metrics = validate.plane_metrics(validate.pose_points(mech, pose), sleepy_pts, sleepy_clearance)
            frame = args.out / f"{name}_{i:02d}.png"
            render_assembly(asm, frame, metrics)
            frame_paths.append(frame)
        imgs = [imageio.imread(p) for p in frame_paths]
        gif = args.out / f"{name}.gif"
        imageio.mimsave(gif, imgs, duration=float(t["duration_s"]) / max(1, frames - 1), loop=0)
        print(f"CAD motion: {gif}")


if __name__ == "__main__":
    main()

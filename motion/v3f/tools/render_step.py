from __future__ import annotations

import argparse
from pathlib import Path

import cadquery as cq
import vtk
from PIL import Image, ImageDraw


def shape_to_polydata(shape, lin_tol=0.7, ang_tol=0.15):
    verts, tris = shape.tessellate(lin_tol, ang_tol)
    pts = vtk.vtkPoints()
    pts.SetNumberOfPoints(len(verts))
    for i, v in enumerate(verts):
        pts.SetPoint(i, v.x, v.y, v.z)
    cells = vtk.vtkCellArray()
    for tri in tris:
        cell = vtk.vtkTriangle()
        for j, idx in enumerate(tri):
            cell.GetPointIds().SetId(j, int(idx))
        cells.InsertNextCell(cell)
    poly = vtk.vtkPolyData()
    poly.SetPoints(pts)
    poly.SetPolys(cells)
    return poly


def render_step(step: Path, out: Path, view: str = "isometric", size=(720, 640), engineering=False):
    shape = cq.importers.importStep(str(step)).val()
    poly = shape_to_polydata(shape)
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(poly)
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(*(0.62, 0.64, 0.67) if engineering else (0.88, 0.86, 0.81))
    actor.GetProperty().SetInterpolationToPhong()

    edges = vtk.vtkFeatureEdges()
    edges.SetInputData(poly)
    edges.BoundaryEdgesOn()
    edges.FeatureEdgesOn()
    edges.ManifoldEdgesOff()
    edges.NonManifoldEdgesOn()
    emap = vtk.vtkPolyDataMapper()
    emap.SetInputConnection(edges.GetOutputPort())
    eactor = vtk.vtkActor()
    eactor.SetMapper(emap)
    eactor.GetProperty().SetColor(0.22, 0.23, 0.24)
    eactor.GetProperty().SetLineWidth(0.7)

    ren = vtk.vtkRenderer()
    ren.SetBackground(0.97, 0.97, 0.965)
    ren.AddActor(actor)
    ren.AddActor(eactor)
    win = vtk.vtkRenderWindow()
    win.SetOffScreenRendering(1)
    win.AddRenderer(ren)
    win.SetSize(*size)
    cam = ren.GetActiveCamera()
    if view == "front":
        cam.SetPosition(300, 0, 85)
    elif view == "side":
        cam.SetPosition(0, -300, 85)
    elif view == "top":
        cam.SetPosition(0, 0, 360)
    else:
        cam.SetPosition(265, -245, 205)
    cam.SetFocalPoint(0, 0, 76)
    cam.SetViewUp(0, 0, 1)
    ren.ResetCameraClippingRange()
    win.Render()
    grab = vtk.vtkWindowToImageFilter()
    grab.SetInput(win)
    grab.Update()
    writer = vtk.vtkPNGWriter()
    writer.SetFileName(str(out))
    writer.SetInputConnection(grab.GetOutputPort())
    writer.Write()


def make_sheet(items, out: Path, title="V3-F CAD-derived sentinel poses"):
    imgs = [(label, Image.open(path).convert("RGB")) for label, path in items]
    if not imgs:
        return
    w = max(im.width for _, im in imgs)
    h = max(im.height for _, im in imgs)
    cols = 2
    rows = (len(imgs) + 1) // 2
    top = 62
    label_h = 34
    sheet = Image.new("RGB", (cols * w, top + rows * (h + label_h)), (248, 248, 246))
    draw = ImageDraw.Draw(sheet)
    draw.text((18, 18), title, fill=(25, 25, 25))
    for i, (label, im) in enumerate(imgs):
        x = (i % cols) * w
        y = top + (i // cols) * (h + label_h)
        sheet.paste(im, (x, y))
        draw.text((x + 14, y + h + 8), label, fill=(30, 30, 30))
    sheet.save(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--generated", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    mapping = [
        ("neutral", "Pebble_V3F_Product_Neutral.step", False),
        ("sleepy_compact", "Pebble_V3F_Inward_Compact.step", False),
        ("happy_wide", "Pebble_V3F_Happy_Wide.step", False),
        ("play_bow", "Pebble_V3F_Play_Bow.step", False),
        ("curious_left", "Pebble_V3F_Curious_Left.step", False),
        ("engineering_cutaway", "Pebble_V3F_Engineering_Cutaway.step", True),
    ]
    rendered = []
    for label, filename, engineering in mapping:
        step = args.generated / filename
        if not step.exists():
            continue
        out = args.out / f"{label}.png"
        render_step(step, out, "isometric", engineering=engineering)
        rendered.append((label, out))
    make_sheet([x for x in rendered if x[0] != "engineering_cutaway"], args.out / "sentinel_pose_sheet.png")
    print(f"Rendered {len(rendered)} CAD views in {args.out}")


if __name__ == "__main__":
    main()

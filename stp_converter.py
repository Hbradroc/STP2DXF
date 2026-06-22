#!/usr/bin/env python3
"""
Convert STEP (.stp/.step) models to DXF files.

Flat DXF: cut-ready 2D outlines extracted from planar plate faces.
Drawing DXF: orthographic 3-view projection with hidden-line removal.
"""

from __future__ import annotations

import argparse
import hashlib
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import ezdxf
from OCP.BRepAdaptor import BRepAdaptor_Curve, BRepAdaptor_Surface
from OCP.HLRAlgo import HLRAlgo_Projector
from OCP.HLRBRep import HLRBRep_Algo, HLRBRep_HLRToShape
from OCP.BRepGProp import BRepGProp
from OCP.BRepTools import BRepTools
from OCP.GCPnts import GCPnts_QuasiUniformDeflection
from OCP.GProp import GProp_GProps
from OCP.GeomAbs import (
    GeomAbs_Circle,
    GeomAbs_Ellipse,
    GeomAbs_Line,
    GeomAbs_Plane,
)
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_WIRE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt, gp_Vec


@dataclass
class PlanarFace:
    face: object
    area: float
    normal: tuple[float, float, float]
    origin: tuple[float, float, float]
    x_axis: tuple[float, float, float]
    y_axis: tuple[float, float, float]
    centroid: tuple[float, float, float]
    uv_width: float
    uv_height: float

    @property
    def min_extent(self) -> float:
        return min(self.uv_width, self.uv_height)


@dataclass
class Point2D:
    x: float
    y: float


@dataclass
class ViewSpec:
    name: str
    normal: tuple[float, float, float]
    x_direction: tuple[float, float, float]


@dataclass
class ViewLayout:
    spec: ViewSpec
    offset_x: float
    offset_y: float
    width: float
    height: float
    visible_edges: object
    hidden_edges: object


DRAWING_VIEWS = (
    ViewSpec("FRONT", (0.0, -1.0, 0.0), (1.0, 0.0, 0.0)),
    ViewSpec("TOP", (0.0, 0.0, -1.0), (1.0, 0.0, 0.0)),
    ViewSpec("RIGHT", (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
)
VIEW_GAP = 50.0


def _vec_xyz(direction) -> tuple[float, float, float]:
    return (direction.X(), direction.Y(), direction.Z())


def _normalize(vec: tuple[float, float, float]) -> tuple[float, float, float]:
    length = math.sqrt(vec[0] ** 2 + vec[1] ** 2 + vec[2] ** 2)
    if length < 1e-12:
        return (0.0, 0.0, 0.0)
    return (vec[0] / length, vec[1] / length, vec[2] / length)


def _dot(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _sub(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _canonical_normal(normal: tuple[float, float, float]) -> tuple[float, float, float]:
    n = _normalize(normal)
    if n[2] < -1e-9 or (abs(n[2]) <= 1e-9 and (n[1] < -1e-9 or (abs(n[1]) <= 1e-9 and n[0] < 0))):
        return (-n[0], -n[1], -n[2])
    return n


def load_step(path: Path):
    reader = STEPControl_Reader()
    status = reader.ReadFile(str(path))
    if status != IFSelect_RetDone:
        raise RuntimeError(f"Failed to read STEP file: {path}")
    reader.TransferRoots()
    return reader.OneShape()


def face_uv_extents(face, origin, x_axis, y_axis) -> tuple[float, float]:
    xs: list[float] = []
    ys: list[float] = []
    exp = TopExp_Explorer(face, TopAbs_EDGE)
    while exp.More():
        edge = TopoDS.Edge_s(exp.Current())
        curve = BRepAdaptor_Curve(edge)
        u1 = curve.FirstParameter()
        u2 = curve.LastParameter()
        for u in (u1, u2, (u1 + u2) * 0.5):
            pnt = curve.Value(u)
            vec = _sub((pnt.X(), pnt.Y(), pnt.Z()), origin)
            xs.append(_dot(vec, x_axis))
            ys.append(_dot(vec, y_axis))
        exp.Next()
    if not xs:
        return 0.0, 0.0
    return max(xs) - min(xs), max(ys) - min(ys)


def collect_planar_faces(
    shape,
    min_area: float,
    thickness: float,
) -> list[PlanarFace]:
    faces: list[PlanarFace] = []
    min_extent = max(thickness * 1.5, 10.0)
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        face = TopoDS.Face_s(exp.Current())
        surf = BRepAdaptor_Surface(face)
        if surf.GetType() != GeomAbs_Plane:
            exp.Next()
            continue

        props = GProp_GProps()
        BRepGProp.SurfaceProperties_s(face, props)
        area = props.Mass()
        if area < min_area:
            exp.Next()
            continue

        pln = surf.Plane()
        origin = _vec_xyz(pln.Location())
        x_axis = _vec_xyz(pln.XAxis().Direction())
        y_axis = _vec_xyz(pln.YAxis().Direction())
        uv_width, uv_height = face_uv_extents(face, origin, x_axis, y_axis)
        if min(uv_width, uv_height) < min_extent:
            exp.Next()
            continue

        normal = _vec_xyz(pln.Axis().Direction())
        if face.Orientation() == 1:  # TopAbs_REVERSED
            normal = (-normal[0], -normal[1], -normal[2])

        cog = props.CentreOfMass()
        centroid = (cog.X(), cog.Y(), cog.Z())

        faces.append(
            PlanarFace(
                face=face,
                area=area,
                normal=normal,
                origin=origin,
                x_axis=x_axis,
                y_axis=y_axis,
                centroid=centroid,
                uv_width=uv_width,
                uv_height=uv_height,
            )
        )
        exp.Next()
    faces.sort(key=lambda f: f.area, reverse=True)
    return faces


def make_projector(face: PlanarFace):
    origin = face.origin
    x_axis = face.x_axis
    y_axis = face.y_axis

    def project(pnt: gp_Pnt) -> Point2D:
        vec = _sub((pnt.X(), pnt.Y(), pnt.Z()), origin)
        return Point2D(
            x=_dot(vec, x_axis),
            y=_dot(vec, y_axis),
        )

    return project


def _angle_deg(center: Point2D, point: Point2D) -> float:
    return math.degrees(math.atan2(point.y - center.y, point.x - center.x))


def discretize_edge(edge, project, chord_tol: float = 0.01) -> list[tuple[str, object]]:
    curve = BRepAdaptor_Curve(edge)
    curve_type = curve.GetType()
    u1 = curve.FirstParameter()
    u2 = curve.LastParameter()

    if curve_type == GeomAbs_Line:
        p1 = project(curve.Value(u1))
        p2 = project(curve.Value(u2))
        return [("line", (p1, p2))]

    if curve_type == GeomAbs_Circle:
        circle = curve.Circle()
        center = project(circle.Location())
        radius = circle.Radius()
        start = project(curve.Value(u1))
        end = project(curve.Value(u2))
        start_angle = _angle_deg(center, start)
        end_angle = _angle_deg(center, end)
        span = abs(end_angle - start_angle)
        if span > 359.5 and abs(start.x - end.x) < 1e-6 and abs(start.y - end.y) < 1e-6:
            return [("circle", (center, radius))]
        return [("arc", (center, radius, start_angle, end_angle))]

    if curve_type == GeomAbs_Ellipse:
        pass

    # Deflection-based tessellation for splines and other free-form curves.
    deflection = GCPnts_QuasiUniformDeflection()
    deflection.Initialize(curve, max(chord_tol, 1e-6))
    if deflection.IsDone() and deflection.NbPoints() >= 2:
        segments: list[tuple[str, object]] = []
        prev = project(deflection.Value(1))
        for i in range(2, deflection.NbPoints() + 1):
            curr = project(deflection.Value(i))
            if math.hypot(curr.x - prev.x, curr.y - prev.y) > 1e-9:
                segments.append(("line", (prev, curr)))
            prev = curr
        return segments

    segments: list[tuple[str, object]] = []
    n = 32
    prev = project(curve.Value(u1))
    for i in range(1, n + 1):
        t = u1 + (u2 - u1) * (i / n)
        curr = project(curve.Value(t))
        if math.hypot(curr.x - prev.x, curr.y - prev.y) > 1e-9:
            segments.append(("line", (prev, curr)))
        prev = curr
    return segments


def wire_edges(wire) -> list:
    edges = []
    exp = TopExp_Explorer(wire, TopAbs_EDGE)
    while exp.More():
        edges.append(TopoDS.Edge_s(exp.Current()))
        exp.Next()
    return edges


def face_wires(face) -> list:
    wires = []
    exp = TopExp_Explorer(face, TopAbs_WIRE)
    while exp.More():
        wires.append(TopoDS.Wire_s(exp.Current()))
        exp.Next()
    return wires


def face_signature(face: PlanarFace, project, precision: int = 3) -> str:
    parts: list[str] = []
    outer = BRepTools.OuterWire_s(face.face)
    for wire in [outer] + [w for w in face_wires(face.face) if not w.IsSame(outer)]:
        for edge in wire_edges(wire):
            curve = BRepAdaptor_Curve(edge)
            u1 = curve.FirstParameter()
            u2 = curve.LastParameter()
            p1 = project(curve.Value(u1))
            p2 = project(curve.Value(u2))
            parts.append(
                f"{round(p1.x, precision)}:{round(p1.y, precision)}-"
                f"{round(p2.x, precision)}:{round(p2.y, precision)}"
            )
    parts.sort()
    digest = hashlib.sha1("|".join(parts).encode("utf-8")).hexdigest()
    return digest


def deduplicate_faces(faces: list[PlanarFace], thickness: float) -> list[PlanarFace]:
    kept: list[PlanarFace] = []
    seen_signatures: set[str] = set()

    for face in faces:
        project = make_projector(face)
        signature = face_signature(face, project)
        if signature in seen_signatures:
            continue

        duplicate = False
        for other in kept:
            if abs(face.area - other.area) / max(face.area, other.area) > 0.002:
                continue
            n1 = _canonical_normal(face.normal)
            n2 = _canonical_normal(other.normal)
            if abs(_dot(n1, n2) + 1.0) > 0.02:
                continue
            dist = abs(_dot(_sub(face.centroid, other.centroid), n1))
            if dist <= thickness:
                duplicate = True
                break

        if duplicate:
            continue

        seen_signatures.add(signature)
        kept.append(face)

    return kept


def add_entities(msp, segments, layer: str) -> None:
    for kind, data in segments:
        if kind == "line":
            p1, p2 = data
            msp.add_line((p1.x, p1.y), (p2.x, p2.y), dxfattribs={"layer": layer})
        elif kind == "circle":
            center, radius = data
            msp.add_circle((center.x, center.y), radius, dxfattribs={"layer": layer})
        elif kind == "arc":
            center, radius, start_angle, end_angle = data
            msp.add_arc(
                (center.x, center.y),
                radius,
                start_angle,
                end_angle,
                dxfattribs={"layer": layer},
            )


def hlr_edges(shape, view: ViewSpec) -> tuple[object, object]:
    axis = gp_Ax2(
        gp_Pnt(0.0, 0.0, 0.0),
        gp_Dir(*view.normal),
        gp_Dir(*view.x_direction),
    )
    hlr = HLRBRep_Algo()
    hlr.Add(shape)
    hlr.Projector(HLRAlgo_Projector(axis))
    hlr.Update()
    hlr.Hide()
    result = HLRBRep_HLRToShape(hlr)
    return result.VCompound(), result.HCompound()


def compound_2d_bbox(compound) -> tuple[float, float, float, float] | None:
    if compound is None or compound.IsNull():
        return None
    xs: list[float] = []
    ys: list[float] = []
    exp = TopExp_Explorer(compound, TopAbs_EDGE)
    while exp.More():
        curve = BRepAdaptor_Curve(TopoDS.Edge_s(exp.Current()))
        for u in (curve.FirstParameter(), curve.LastParameter()):
            pnt = curve.Value(u)
            xs.append(pnt.X())
            ys.append(pnt.Y())
        exp.Next()
    if not xs:
        return None
    return min(xs), min(ys), max(xs), max(ys)


def project_hlr_point(pnt: gp_Pnt) -> Point2D:
    return Point2D(x=pnt.X(), y=pnt.Y())


def export_compound_edges(
    msp,
    compound,
    offset_x: float,
    offset_y: float,
    layer: str,
    chord_tol: float,
) -> None:
    if compound is None or compound.IsNull():
        return

    def project(pnt: gp_Pnt) -> Point2D:
        point = project_hlr_point(pnt)
        return Point2D(x=point.x + offset_x, y=point.y + offset_y)

    exp = TopExp_Explorer(compound, TopAbs_EDGE)
    while exp.More():
        edge = TopoDS.Edge_s(exp.Current())
        segments = discretize_edge(edge, project, chord_tol=chord_tol)
        add_entities(msp, segments, layer)
        exp.Next()


def layout_drawing_views(shape) -> list[ViewLayout]:
    raw_views: list[tuple[ViewSpec, object, object, tuple[float, float, float, float]]] = []
    for spec in DRAWING_VIEWS:
        visible, hidden = hlr_edges(shape, spec)
        bbox = compound_2d_bbox(visible)
        if bbox is None:
            bbox = compound_2d_bbox(hidden)
        if bbox is None:
            continue
        raw_views.append((spec, visible, hidden, bbox))

    if not raw_views:
        raise RuntimeError("Could not generate any drawing views from the STEP model.")

    by_name = {spec.name: (spec, visible, hidden, bbox) for spec, visible, hidden, bbox in raw_views}
    front = by_name.get("FRONT")
    top = by_name.get("TOP")
    right = by_name.get("RIGHT")
    if front is None:
        front = raw_views[0]
        top = raw_views[1] if len(raw_views) > 1 else None
        right = raw_views[2] if len(raw_views) > 2 else None

    layouts: list[ViewLayout] = []

    def make_layout(spec, visible, hidden, bbox, offset_x, offset_y) -> ViewLayout:
        xmin, ymin, xmax, ymax = bbox
        return ViewLayout(
            spec=spec,
            offset_x=offset_x - xmin,
            offset_y=offset_y - ymin,
            width=xmax - xmin,
            height=ymax - ymin,
            visible_edges=visible,
            hidden_edges=hidden,
        )

    _, front_visible, front_hidden, front_bbox = front
    front_x = 0.0
    front_y = 0.0
    layouts.append(make_layout(front[0], front_visible, front_hidden, front_bbox, front_x, front_y))

    if top is not None:
        _, top_visible, top_hidden, top_bbox = top
        top_x = front_x + (front_bbox[2] - front_bbox[0] - (top_bbox[2] - top_bbox[0])) * 0.5
        top_y = front_y + (front_bbox[3] - front_bbox[1]) + VIEW_GAP
        layouts.append(make_layout(top[0], top_visible, top_hidden, top_bbox, top_x, top_y))

    if right is not None:
        _, right_visible, right_hidden, right_bbox = right
        right_x = front_x + (front_bbox[2] - front_bbox[0]) + VIEW_GAP
        right_y = front_y + (front_bbox[3] - front_bbox[1] - (right_bbox[3] - right_bbox[1])) * 0.5
        layouts.append(make_layout(right[0], right_visible, right_hidden, right_bbox, right_x, right_y))

    return layouts


def export_drawing_to_dxf(shape, output_path: Path, chord_tol: float) -> None:
    doc = ezdxf.new("R2010")
    msp = doc.modelspace()

    if "DASHED" not in doc.linetypes:
        doc.linetypes.add("DASHED", pattern=[0.5, -0.25])

    for view in layout_drawing_views(shape):
        visible_layer = f"{view.spec.name}_VISIBLE"
        hidden_layer = f"{view.spec.name}_HIDDEN"
        doc.layers.add(visible_layer, color=7)
        doc.layers.add(hidden_layer, color=8, linetype="DASHED")
        export_compound_edges(
            msp,
            view.visible_edges,
            view.offset_x,
            view.offset_y,
            visible_layer,
            chord_tol,
        )
        export_compound_edges(
            msp,
            view.hidden_edges,
            view.offset_x,
            view.offset_y,
            hidden_layer,
            chord_tol,
        )

    doc.saveas(output_path)


def export_face_to_dxf(face: PlanarFace, output_path: Path, chord_tol: float) -> None:
    project = make_projector(face)
    doc = ezdxf.new("R2010")
    msp = doc.modelspace()
    doc.layers.add("CUT", color=1)

    outer = BRepTools.OuterWire_s(face.face)
    all_wires = face_wires(face.face)

    for wire in all_wires:
        layer = "CUT"
        for edge in wire_edges(wire):
            segments = discretize_edge(edge, project, chord_tol=chord_tol)
            add_entities(msp, segments, layer)

    doc.saveas(output_path)


def export_step_to_dxf(
    step_path: Path,
    output_dir: Path,
    *,
    min_area: float = 100.0,
    thickness: float = 6.0,
    chord_tol: float = 0.01,
    deduplicate: bool = True,
    verbose: bool = False,
    export_flat: bool = True,
    export_drawing: bool = True,
) -> list[Path]:
    shape = load_step(step_path)
    output_dir.mkdir(parents=True, exist_ok=True)
    stem = step_path.stem
    written: list[Path] = []

    if export_flat:
        faces = collect_planar_faces(shape, min_area=min_area, thickness=thickness)
        if not faces:
            raise RuntimeError(
                "No cuttable planar faces found. Try lowering --min-area or --thickness "
                "if the model uses thinner sheet stock."
            )

        if deduplicate:
            faces = deduplicate_faces(faces, thickness=thickness)

        for index, face in enumerate(faces, start=1):
            suffix = f"_{index:02d}" if len(faces) > 1 else ""
            out_path = output_dir / f"{stem}{suffix}.dxf"
            if verbose:
                print(
                    f"  flat face {index}: area={face.area:.1f} mm2, "
                    f"size={face.uv_width:.1f}x{face.uv_height:.1f} mm, "
                    f"normal=({face.normal[0]:.3f}, {face.normal[1]:.3f}, {face.normal[2]:.3f})"
                )
            export_face_to_dxf(face, out_path, chord_tol=chord_tol)
            written.append(out_path)

    if export_drawing:
        drawing_path = output_dir / f"{stem}_drawing.dxf"
        if verbose:
            print("  drawing: FRONT, TOP, RIGHT orthographic views")
        export_drawing_to_dxf(shape, drawing_path, chord_tol=chord_tol)
        written.append(drawing_path)

    if not written:
        raise RuntimeError("Nothing exported. Enable flat and/or drawing output.")

    return written


def convert_step(
    input_path: Path,
    output_dir: Path,
    *,
    min_area: float = 100.0,
    thickness: float = 6.0,
    chord_tol: float = 0.01,
    deduplicate: bool = True,
    verbose: bool = False,
    export_flat: bool = True,
    export_drawing: bool = True,
) -> dict:
    """
    Convert a STEP file to flat and/or drawing DXF outputs.

    Returns a dict with keys:
      outputs: list[Path]
      flat_face_count: int
    """
    input_path = Path(input_path).resolve()
    output_dir = Path(output_dir).resolve()

    written = export_step_to_dxf(
        input_path,
        output_dir,
        min_area=min_area,
        thickness=thickness,
        chord_tol=chord_tol,
        deduplicate=deduplicate,
        verbose=verbose,
        export_flat=export_flat,
        export_drawing=export_drawing,
    )

    flat_count = sum(1 for path in written if not path.name.endswith("_drawing.dxf"))
    if not export_flat:
        flat_count = 0
    elif export_drawing:
        flat_count = max(0, len(written) - 1)

    return {
        "outputs": written,
        "flat_face_count": flat_count,
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert STEP files to flat cut DXF and/or drawing DXF.",
    )
    parser.add_argument("step_file", type=Path, help="Input .stp/.step file")
    parser.add_argument(
        "-o",
        "--output-dir",
        type=Path,
        default=None,
        help="Output directory (default: <step_file>_dxf/)",
    )
    parser.add_argument(
        "--min-area",
        type=float,
        default=100.0,
        metavar="MM2",
        help="Minimum planar face area to export in mm2 (default: 100)",
    )
    parser.add_argument(
        "-t",
        "--thickness",
        type=float,
        default=6.0,
        metavar="MM",
        help="Expected sheet thickness in mm; filters edge faces and deduplicates pairs (default: 6)",
    )
    parser.add_argument(
        "--chord-tol",
        type=float,
        default=0.01,
        metavar="MM",
        help="Tessellation tolerance for free-form curves (default: 0.01)",
    )
    parser.add_argument(
        "--no-deduplicate",
        action="store_true",
        help="Export both sides of each plate (skip top/bottom deduplication)",
    )
    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="Print details about each exported face",
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--flat-only",
        action="store_true",
        help="Export only flat cut-ready DXF files",
    )
    mode.add_argument(
        "--drawing-only",
        action="store_true",
        help="Export only the orthographic drawing DXF",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    step_path = args.step_file.resolve()
    if not step_path.exists():
        print(f"Error: file not found: {step_path}", file=sys.stderr)
        return 1

    output_dir = args.output_dir
    if output_dir is None:
        output_dir = step_path.parent / f"{step_path.stem}_dxf"
    output_dir = output_dir.resolve()

    try:
        if args.verbose:
            print(f"Reading {step_path.name}...")
        result = convert_step(
            step_path,
            output_dir,
            min_area=args.min_area,
            thickness=args.thickness,
            chord_tol=args.chord_tol,
            deduplicate=not args.no_deduplicate,
            verbose=args.verbose,
            export_flat=not args.drawing_only,
            export_drawing=not args.flat_only,
        )
        written = result["outputs"]
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    print(f"Exported {len(written)} DXF file(s) to {output_dir}:")
    for path in written:
        print(f"  {path.name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

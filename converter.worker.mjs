/**
 * STEP geometry extraction for the browser (OpenCascade via replicad-opencascadejs).
 * Returns layer/entity specs consumed by stp_converter.write_dxf_from_specs in Pyodide.
 */

const OCCT_BASE = "https://cdn.jsdelivr.net/npm/replicad-opencascadejs@0.23.0/src/";

let ocPromise = null;

async function getOc() {
  if (!ocPromise) {
    ocPromise = import(`${OCCT_BASE}replicad_single.js`).then((mod) =>
      mod.default({
        locateFile: (path) => `${OCCT_BASE}${path}`,
      }),
    );
  }
  return ocPromise;
}

const VIEW_GAP = 50;
const DRAWING_VIEWS = [
  { name: "FRONT", normal: [0, -1, 0], xdir: [1, 0, 0] },
  { name: "TOP", normal: [0, 0, -1], xdir: [1, 0, 0] },
  { name: "RIGHT", normal: [1, 0, 0], xdir: [0, 1, 0] },
];

function vecDot(a, b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function vecSub(a, b) {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

function vecNorm(v) {
  const len = Math.hypot(v[0], v[1], v[2]);
  if (len < 1e-12) return [0, 0, 0];
  return [v[0] / len, v[1] / len, v[2] / len];
}

function canonicalNormal(n) {
  const v = vecNorm(n);
  if (
    v[2] < -1e-9 ||
    (Math.abs(v[2]) <= 1e-9 && (v[1] < -1e-9 || (Math.abs(v[1]) <= 1e-9 && v[0] < 0)))
  ) {
    return [-v[0], -v[1], -v[2]];
  }
  return v;
}

function dirXYZ(d) {
  return [d.X(), d.Y(), d.Z()];
}

function pntXYZ(p) {
  return [p.X(), p.Y(), p.Z()];
}

function angleDeg(cx, cy, px, py) {
  return (Math.atan2(py - cy, px - cx) * 180) / Math.PI;
}

function loadStep(oc, bytes) {
  oc.FS.writeFile("/input.stp", bytes);
  const reader = new oc.STEPControl_Reader_1();
  const status = reader.ReadFile("/input.stp");
  if (status !== oc.IFSelect_ReturnStatus.IFSelect_RetDone) {
    throw new Error("Failed to read STEP file.");
  }
  reader.TransferRoots(new oc.Message_ProgressRange_1());
  return reader.OneShape();
}

function exploreEdges(oc, shape) {
  const edges = [];
  const exp = new oc.TopExp_Explorer_2(
    shape,
    oc.TopAbs_ShapeEnum.TopAbs_EDGE,
    oc.TopAbs_ShapeEnum.TopAbs_SHAPE,
  );
  while (exp.More()) {
    edges.push(oc.TopoDS.Edge_1(exp.Current()));
    exp.Next();
  }
  return edges;
}

function faceUvExtents(oc, face, origin, xAxis, yAxis) {
  const xs = [];
  const ys = [];
  for (const edge of exploreEdges(oc, face)) {
    const curve = new oc.BRepAdaptor_Curve_2(edge, true);
    const u1 = curve.FirstParameter();
    const u2 = curve.LastParameter();
    for (const u of [u1, u2, (u1 + u2) * 0.5]) {
      const p = curve.Value(u);
      const vec = vecSub(pntXYZ(p), origin);
      xs.push(vecDot(vec, xAxis));
      ys.push(vecDot(vec, yAxis));
    }
    curve.delete();
  }
  if (!xs.length) return [0, 0];
  return [Math.max(...xs) - Math.min(...xs), Math.max(...ys) - Math.min(...ys)];
}

function collectPlanarFaces(oc, shape, minArea, thickness) {
  const minExtent = Math.max(thickness * 1.5, 10);
  const faces = [];
  const exp = new oc.TopExp_Explorer_2(
    shape,
    oc.TopAbs_ShapeEnum.TopAbs_FACE,
    oc.TopAbs_ShapeEnum.TopAbs_SHAPE,
  );
  while (exp.More()) {
    const face = oc.TopoDS.Face_1(exp.Current());
    const surf = new oc.BRepAdaptor_Surface_2(face, true);
    if (surf.GetType() !== oc.GeomAbs_SurfaceType.GeomAbs_Plane) {
      surf.delete();
      exp.Next();
      continue;
    }

    const props = new oc.GProp_GProps_1();
    oc.BRepGProp.SurfaceProperties_1(face, props, true, false);
    const area = props.Mass();
    if (area < minArea) {
      props.delete();
      surf.delete();
      exp.Next();
      continue;
    }

    const pln = surf.Plane();
    const origin = pntXYZ(pln.Location());
    const xAxis = dirXYZ(pln.XAxis().Direction());
    const yAxis = dirXYZ(pln.YAxis().Direction());
    const [uvW, uvH] = faceUvExtents(oc, face, origin, xAxis, yAxis);
    if (Math.min(uvW, uvH) < minExtent) {
      props.delete();
      surf.delete();
      exp.Next();
      continue;
    }

    let normal = dirXYZ(pln.Axis().Direction());
    if (face.Orientation_1() === oc.TopAbs_Orientation.TopAbs_REVERSED) {
      normal = [-normal[0], -normal[1], -normal[2]];
    }
    const cog = props.CentreOfMass();

    faces.push({
      face,
      area,
      normal,
      origin,
      xAxis,
      yAxis,
      centroid: pntXYZ(cog),
      uvWidth: uvW,
      uvHeight: uvH,
    });
    props.delete();
    surf.delete();
    exp.Next();
  }
  faces.sort((a, b) => b.area - a.area);
  return faces;
}

function makeProjector(origin, xAxis, yAxis) {
  return (pnt) => {
    const vec = vecSub(pntXYZ(pnt), origin);
    return { x: vecDot(vec, xAxis), y: vecDot(vec, yAxis) };
  };
}

function discretizeEdge(oc, edge, project, chordTol) {
  const curve = new oc.BRepAdaptor_Curve_2(edge, true);
  const curveType = curve.GetType();
  const u1 = curve.FirstParameter();
  const u2 = curve.LastParameter();
  const segments = [];

  if (curveType === oc.GeomAbs_CurveType.GeomAbs_Line) {
    const p1 = project(curve.Value(u1));
    const p2 = project(curve.Value(u2));
    segments.push({ type: "line", x1: p1.x, y1: p1.y, x2: p2.x, y2: p2.y });
  } else if (curveType === oc.GeomAbs_CurveType.GeomAbs_Circle) {
    const circle = curve.Circle();
    const center = project(circle.Location());
    const radius = circle.Radius();
    const start = project(curve.Value(u1));
    const end = project(curve.Value(u2));
    const startAngle = angleDeg(center.x, center.y, start.x, start.y);
    const endAngle = angleDeg(center.x, center.y, end.x, end.y);
    const span = Math.abs(endAngle - startAngle);
    if (span > 359.5 && Math.hypot(start.x - end.x, start.y - end.y) < 1e-6) {
      segments.push({ type: "circle", cx: center.x, cy: center.y, r: radius });
    } else {
      segments.push({
        type: "arc",
        cx: center.x,
        cy: center.y,
        r: radius,
        start: startAngle,
        end: endAngle,
      });
    }
  } else {
    const n = 32;
    let prev = project(curve.Value(u1));
    for (let i = 1; i <= n; i += 1) {
      const t = u1 + ((u2 - u1) * i) / n;
      const curr = project(curve.Value(t));
      if (Math.hypot(curr.x - prev.x, curr.y - prev.y) > 1e-9) {
        segments.push({ type: "line", x1: prev.x, y1: prev.y, x2: curr.x, y2: curr.y });
      }
      prev = curr;
    }
  }

  curve.delete();
  return segments;
}

function faceSignature(oc, faceInfo, project) {
  const parts = [];
  const outer = oc.BRepTools.OuterWire(faceInfo.face);
  const wires = [outer];
  const wireExp = new oc.TopExp_Explorer_2(
    faceInfo.face,
    oc.TopAbs_ShapeEnum.TopAbs_WIRE,
    oc.TopAbs_ShapeEnum.TopAbs_SHAPE,
  );
  while (wireExp.More()) {
    const wire = oc.TopoDS.Wire_1(wireExp.Current());
    if (!wire.IsSame(outer)) wires.push(wire);
    wireExp.Next();
  }

  for (const wire of wires) {
    for (const edge of exploreEdges(oc, wire)) {
      const curve = new oc.BRepAdaptor_Curve_2(edge, true);
      const u1 = curve.FirstParameter();
      const u2 = curve.LastParameter();
      const p1 = project(curve.Value(u1));
      const p2 = project(curve.Value(u2));
      parts.push(
        `${p1.x.toFixed(3)}:${p1.y.toFixed(3)}-${p2.x.toFixed(3)}:${p2.y.toFixed(3)}`,
      );
      curve.delete();
    }
  }
  parts.sort();
  return parts.join("|");
}

function deduplicateFaces(oc, faces, thickness) {
  const kept = [];
  const seen = new Set();
  for (const face of faces) {
    const project = makeProjector(face.origin, face.xAxis, face.yAxis);
    const signature = faceSignature(oc, face, project);
    if (seen.has(signature)) continue;

    let duplicate = false;
    for (const other of kept) {
      if (Math.abs(face.area - other.area) / Math.max(face.area, other.area) > 0.002) continue;
      const n1 = canonicalNormal(face.normal);
      const n2 = canonicalNormal(other.normal);
      if (Math.abs(vecDot(n1, n2) + 1) > 0.02) continue;
      const dist = Math.abs(vecDot(vecSub(face.centroid, other.centroid), n1));
      if (dist <= thickness) {
        duplicate = true;
        break;
      }
    }
    if (duplicate) continue;
    seen.add(signature);
    kept.push(face);
  }
  return kept;
}

function faceToSpec(oc, faceInfo, layerName) {
  const project = makeProjector(faceInfo.origin, faceInfo.xAxis, faceInfo.yAxis);
  const entities = [];
  const wireExp = new oc.TopExp_Explorer_2(
    faceInfo.face,
    oc.TopAbs_ShapeEnum.TopAbs_WIRE,
    oc.TopAbs_ShapeEnum.TopAbs_SHAPE,
  );
  while (wireExp.More()) {
    for (const edge of exploreEdges(oc, oc.TopoDS.Wire_1(wireExp.Current()))) {
      entities.push(...discretizeEdge(oc, edge, project, 0.01));
    }
    wireExp.Next();
  }
  return {
    layers: { [layerName]: entities },
    layer_colors: { [layerName]: 1 },
  };
}

function compoundBBox(oc, compound) {
  if (!compound || compound.IsNull()) return null;
  const xs = [];
  const ys = [];
  for (const edge of exploreEdges(oc, compound)) {
    const curve = new oc.BRepAdaptor_Curve_2(edge, true);
    for (const u of [curve.FirstParameter(), curve.LastParameter()]) {
      const p = curve.Value(u);
      xs.push(p.X());
      ys.push(p.Y());
    }
    curve.delete();
  }
  if (!xs.length) return null;
  return [Math.min(...xs), Math.min(...ys), Math.max(...xs), Math.max(...ys)];
}

function makeViewAxis2(oc, normal, xdir) {
  return new oc.gp_Ax2_2(
    new oc.gp_Pnt_1(),
    new oc.gp_Dir_4(normal[0], normal[1], normal[2]),
    new oc.gp_Dir_4(xdir[0], xdir[1], xdir[2]),
  );
}

function hlrEdges(oc, shape, view) {
  const ax = makeViewAxis2(oc, view.normal, view.xdir);
  const hlr = new oc.HLRBRep_Algo_1();
  hlr.Add_2(shape, 0);
  hlr.Projector_1(new oc.HLRAlgo_Projector_2(ax));
  hlr.Update();
  hlr.Hide_1();
  const result = new oc.HLRBRep_HLRToShape(new oc.Handle_HLRBRep_Algo_2(hlr));
  return {
    visible: result.VCompound_1(),
    hidden: result.HCompound_1(),
  };
}

function exportCompound(oc, compound, offsetX, offsetY) {
  const entities = [];
  if (!compound || compound.IsNull()) return entities;
  const project = (pnt) => ({ x: pnt.X() + offsetX, y: pnt.Y() + offsetY });
  for (const edge of exploreEdges(oc, compound)) {
    entities.push(...discretizeEdge(oc, edge, project, 0.01));
  }
  return entities;
}

function layoutDrawingSpecs(oc, shape) {
  const raw = [];
  for (const view of DRAWING_VIEWS) {
    const { visible, hidden } = hlrEdges(oc, shape, view);
    const bbox = compoundBBox(oc, visible) || compoundBBox(oc, hidden);
    if (!bbox) continue;
    raw.push({ view, visible, hidden, bbox });
  }
  if (!raw.length) throw new Error("Could not generate drawing views.");

  const byName = Object.fromEntries(raw.map((item) => [item.view.name, item]));
  const front = byName.FRONT || raw[0];
  const top = byName.TOP;
  const right = byName.RIGHT;
  const layouts = [];

  const makeLayout = (item, offsetX, offsetY) => {
    const [xmin, ymin, xmax, ymax] = item.bbox;
    return {
      view: item.view.name,
      offsetX: offsetX - xmin,
      offsetY: offsetY - ymin,
      visible: item.visible,
      hidden: item.hidden,
      width: xmax - xmin,
      height: ymax - ymin,
    };
  };

  const [, , fxmax, fymax] = front.bbox;
  const [fxmin, fymin] = front.bbox;
  layouts.push(makeLayout(front, 0, 0));

  if (top) {
    const [txmin, , txmax] = top.bbox;
    layouts.push(
      makeLayout(top, (fxmax - fxmin - (txmax - txmin)) * 0.5, fymax - fymin + VIEW_GAP),
    );
  }
  if (right) {
    const [, rymin, , rymax] = right.bbox;
    layouts.push(
      makeLayout(right, fxmax - fxmin + VIEW_GAP, (fymax - fymin - (rymax - rymin)) * 0.5),
    );
  }
  return layouts;
}

function drawingSpec(oc, shape) {
  const layers = {};
  const layerColors = {};
  const layerLinetypes = {};
  for (const layout of layoutDrawingSpecs(oc, shape)) {
    const visibleLayer = `${layout.view}_VISIBLE`;
    const hiddenLayer = `${layout.view}_HIDDEN`;
    layers[visibleLayer] = exportCompound(oc, layout.visible, layout.offsetX, layout.offsetY);
    layers[hiddenLayer] = exportCompound(oc, layout.hidden, layout.offsetX, layout.offsetY);
    layerColors[visibleLayer] = 7;
    layerColors[hiddenLayer] = 8;
    layerLinetypes[hiddenLayer] = "DASHED";
  }
  return { layers, layer_colors: layerColors, layer_linetypes: layerLinetypes };
}

function extractSpecs(oc, bytes, options) {
  const shape = loadStep(oc, bytes);
  const specs = [];
  const stem = options.stem || "converted";

  if (options.exportFlat) {
    let faces = collectPlanarFaces(oc, shape, options.minArea, options.thickness);
    if (!faces.length) {
      throw new Error(
        "No cuttable planar faces found. Try lowering thickness or min-area.",
      );
    }
    if (options.deduplicate) faces = deduplicateFaces(oc, faces, options.thickness);
    faces.forEach((faceInfo, index) => {
      const suffix = faces.length > 1 ? `_${String(index + 1).padStart(2, "0")}` : "";
      specs.push({
        name: `${stem}${suffix}.dxf`,
        ...faceToSpec(oc, faceInfo, "CUT"),
      });
    });
  }

  if (options.exportDrawing) {
    specs.push({
      name: `${stem}_drawing.dxf`,
      ...drawingSpec(oc, shape),
    });
  }

  if (!specs.length) throw new Error("Nothing to export.");
  return {
    specs,
    flatFaceCount: options.exportFlat
      ? specs.filter((s) => !s.name.endsWith("_drawing.dxf")).length
      : 0,
  };
}

self.onmessage = async (event) => {
  try {
    const { bytes, options } = event.data;
    const oc = await getOc();
    const result = extractSpecs(oc, new Uint8Array(bytes), options);
    self.postMessage({ ok: true, ...result });
  } catch (error) {
    self.postMessage({
      ok: false,
      error: error?.message || String(error),
    });
  }
};

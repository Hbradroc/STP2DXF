"""
Pure-Python DXF writer for browser (Pyodide) and CLI.

No OpenCascade imports — safe to load in Pyodide alongside ezdxf.
"""

from __future__ import annotations

from io import StringIO

import ezdxf


def write_dxf_from_specs(specs: list[dict]) -> list[dict]:
    """
    Write DXF files from geometry specs produced by the browser worker.

    Each spec:
      {
        "name": "part.dxf",
        "layers": {
          "CUT": [{"type":"line","x1":..,"y1":..,"x2":..,"y2":..}, ...],
          ...
        }
      }
    """
    written: list[dict] = []
    for spec in specs:
        doc = ezdxf.new("R2010")
        msp = doc.modelspace()
        if "DASHED" not in doc.linetypes:
            doc.linetypes.add("DASHED", pattern=[0.5, -0.25])

        for layer_name, entities in spec.get("layers", {}).items():
            linetype = spec.get("layer_linetypes", {}).get(layer_name)
            color = spec.get("layer_colors", {}).get(layer_name, 7)
            if linetype:
                doc.layers.add(layer_name, color=color, linetype=linetype)
            else:
                doc.layers.add(layer_name, color=color)

            for entity in entities:
                kind = entity["type"]
                if kind == "line":
                    msp.add_line(
                        (entity["x1"], entity["y1"]),
                        (entity["x2"], entity["y2"]),
                        dxfattribs={"layer": layer_name},
                    )
                elif kind == "circle":
                    msp.add_circle(
                        (entity["cx"], entity["cy"]),
                        entity["r"],
                        dxfattribs={"layer": layer_name},
                    )
                elif kind == "arc":
                    msp.add_arc(
                        (entity["cx"], entity["cy"]),
                        entity["r"],
                        entity["start"],
                        entity["end"],
                        dxfattribs={"layer": layer_name},
                    )

        stream = StringIO()
        doc.write(stream)
        written.append(
            {"name": spec["name"], "content": stream.getvalue().encode("utf-8")}
        )
    return written


def write_dxf_from_specs_for_js(specs: list[dict]) -> list[dict]:
    """Like write_dxf_from_specs but with byte lists for Pyodide/JS interop."""
    return [
        {"name": item["name"], "content": list(item["content"])}
        for item in write_dxf_from_specs(specs)
    ]

# STP2DXF

Convert **STEP/STP** 3D models to **flat cut DXF** and **orthographic drawing DXF** — for laser, plasma, waterjet, and documentation workflows.

Available as a **[web app on GitHub Pages](https://hbradroc.github.io/STP2DXF/)** (upload in browser, no install) and as a **command-line tool**.

Companion project: [DXF2CAD](https://github.com/Hbradroc/DXF2CAD) (3D DXF → STEP/STL/IGES).

---

## Outputs

| File | Description |
|------|-------------|
| `<name>.dxf` | Flat cut-ready outline (planar plate faces, holes included) |
| `<name>_drawing.dxf` | Regular drawing DXF — front, top, right views with hidden lines dashed |

---

## Web App (GitHub Pages)

Open **[hbradroc.github.io/STP2DXF](https://hbradroc.github.io/STP2DXF/)** — upload a STEP file, pick options, download DXF files. No install required.

| File | Role |
|------|------|
| `index.html` | Upload UI |
| `app.js` | Orchestrates OpenCascade WASM worker + Pyodide/ezdxf DXF writer |
| `converter.worker.mjs` | Reads STEP, extracts geometry (OpenCascade in browser) |
| `dxf_writer.py` | Pyodide-safe DXF writer (ezdxf only, no OpenCascade) |
| `stp_converter.py` | Full STEP converter for CLI (requires cadquery-ocp) |
| `styles.css` | Layout |

**Note:** First visit loads OpenCascade (~15–25 MB) and Pyodide/ezdxf (~30s). Conversion runs locally — your STEP file is never uploaded to a server.

### Enable GitHub Pages (one time)

1. Push this repo to GitHub.
2. **Settings → Pages → Source: Deploy from branch → `main` / (root) → Save**
3. Open `https://hbradroc.github.io/STP2DXF/`

### Test locally

```bash
python -m http.server 8080
# open http://localhost:8080
```

---

## Installation & Setup (CLI)

### Prerequisites

- Python 3.9+ (3.13 recommended)
- OpenCascade bindings via `cadquery-ocp`

### Install

```bash
pip install -r requirements.txt
```

---

## CLI Usage

```bash
python stp2dxf.py model.stp -t 7 -v
```

| Argument | Default | Description |
|----------|---------|-------------|
| `input.stp` | required | Input STEP file |
| `-o DIR` | `<name>_dxf/` | Output directory |
| `-t MM` | `6` | Sheet thickness — filters edge faces, deduplicates pairs |
| `--min-area` | `100` | Minimum flat face area (mm²) |
| `--flat-only` | off | Skip drawing DXF |
| `--drawing-only` | off | Skip flat cut DXF |
| `--no-deduplicate` | off | Export both sides of each plate |
| `-v` | off | Verbose output |

### Examples

```bash
# Flat cut + drawing DXF (default)
python stp2dxf.py part.stp -t 7

# Flat cut only
python stp2dxf.py part.stp --flat-only -t 7

# Drawing views only
python stp2dxf.py part.stp --drawing-only
```

---

## How It Works

```
Input STEP
    ↓
[1] Load with OpenCascade STEPControl_Reader
    ↓
[2] Flat DXF — find cuttable planar faces
    • Filter thin edge bands by sheet thickness
    • Deduplicate top/bottom plate pairs
    • Project edges → lines, arcs, circles
    ↓
[3] Drawing DXF — HLR orthographic views
    • Front, top, right (third-angle layout)
    • Visible + hidden (dashed) layers
    ↓
Output DXF file(s)
```

---

## Troubleshooting

### "No cuttable planar faces found"

The model may use thinner sheet than `-t` assumes, or has no large flat faces. Try `-t 3` or `--min-area 10`.

### Web app blocked on corporate network

The page loads runtimes from `cdn.pyodide.org`, `cdn.jsdelivr.net`, and `replicad-opencascadejs` on jsDelivr. If blocked, use VPN or the CLI locally.

### Bent sheet metal vs flat pattern

This tool exports **each flat panel face** and **drawing views** of the 3D model. It does **not** unfold sheet metal into a single developed blank.

---

## License

MIT License — free for commercial and personal use.

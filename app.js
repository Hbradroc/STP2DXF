/**
 * STP2DXF — browser conversion via OpenCascade WASM + Pyodide/ezdxf (GitHub Pages).
 */

const PYODIDE_CDNS = [
  "https://cdn.pyodide.org/v0.26.4/full/",
  "https://cdn.jsdelivr.net/pyodide/v0.26.4/full/",
];
const CONVERTER_PATH = "./dxf_writer.py";

const stepInput = document.getElementById("stepFile");
const thicknessInput = document.getElementById("thickness");
const minAreaInput = document.getElementById("minArea");
const exportFlatInput = document.getElementById("exportFlat");
const exportDrawingInput = document.getElementById("exportDrawing");
const deduplicateInput = document.getElementById("deduplicate");
const convertBtn = document.getElementById("convertBtn");
const logEl = document.getElementById("log");
const downloadsEl = document.getElementById("downloads");

let pyodide = null;
let runtimeReady = false;
let runtimePromise = null;
let worker = null;

function log(message) {
  logEl.textContent += `${message}\n`;
  logEl.scrollTop = logEl.scrollHeight;
}

function clearLog() {
  logEl.textContent = "";
}

function clearDownloads() {
  downloadsEl.innerHTML = "";
  downloadsEl.hidden = true;
}

function downloadBlob(filename, bytes) {
  const blob = new Blob([bytes], { type: "application/dxf" });
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = filename;
  a.textContent = `Download ${filename}`;
  downloadsEl.appendChild(a);
  downloadsEl.hidden = false;
}

function stemFromFilename(name) {
  const base = name.replace(/\\/g, "/").split("/").pop() || "converted";
  const dot = base.lastIndexOf(".");
  return dot > 0 ? base.slice(0, dot) : base;
}

function getWorker() {
  if (!worker) {
    worker = new Worker("./converter.worker.mjs", { type: "module" });
  }
  return worker;
}

function extractGeometry(bytes, options) {
  return new Promise((resolve, reject) => {
    const w = getWorker();
    const handler = (event) => {
      w.removeEventListener("message", handler);
      if (event.data.ok) {
        resolve(event.data);
      } else {
        reject(new Error(event.data.error || "Geometry extraction failed."));
      }
    };
    w.addEventListener("message", handler);
    w.postMessage({ bytes, options });
  });
}

async function ensureRuntime() {
  if (runtimeReady) {
    return pyodide;
  }
  if (runtimePromise) {
    return runtimePromise;
  }

  runtimePromise = (async () => {
    log("Loading Python runtime for DXF export (first visit may take ~20s)...");

    const detectedBase = window.__pyodideIndexURL;
    const basesToTry = detectedBase
      ? [detectedBase, ...PYODIDE_CDNS.filter((b) => b !== detectedBase)]
      : PYODIDE_CDNS;

    let lastErr = null;
    for (const base of basesToTry) {
      try {
        const host = new URL(base).hostname;
        log(`Trying Pyodide from ${host}…`);
        pyodide = await loadPyodide({ indexURL: base });
        log(`Pyodide loaded from ${host}.`);
        break;
      } catch (error) {
        lastErr = error;
        log(` blocked or failed: ${error.message?.split("\n")[0] ?? error}`);
        pyodide = null;
      }
    }

    if (!pyodide) {
      throw new Error(
        "Python runtime could not be loaded from any CDN.\n" +
          "Try a different network or VPN.\n" +
          `Last error: ${lastErr?.message ?? lastErr}`,
      );
    }

    await pyodide.loadPackage("micropip");
    log("Installing ezdxf...");
    await pyodide.runPythonAsync(`
import micropip
await micropip.install("ezdxf")
`);

    const converterSource = await fetch(CONVERTER_PATH, { cache: "no-store" }).then((response) => {
      if (!response.ok) {
        throw new Error("Could not load dxf_writer.py from this site.");
      }
      return response.text();
    });
    pyodide.FS.writeFile("/dxf_writer.py", converterSource);

    log("Loading DXF writer module...");
    await pyodide.runPythonAsync(`
import sys
if "/" not in sys.path:
    sys.path.insert(0, "/")
import importlib
import dxf_writer
importlib.reload(dxf_writer)
`);

    runtimeReady = true;
    log("DXF writer ready.\n");
    return pyodide;
  })();

  return runtimePromise;
}

convertBtn.addEventListener("click", async () => {
  clearLog();
  clearDownloads();

  const file = stepInput.files?.[0];
  if (!file) {
    log("Please upload a .stp/.step file.");
    return;
  }

  if (!exportFlatInput.checked && !exportDrawingInput.checked) {
    log("Select at least one output type.");
    return;
  }

  const stem = stemFromFilename(file.name);
  const options = {
    stem,
    thickness: Number(thicknessInput.value) || 6,
    minArea: Number(minAreaInput.value) || 100,
    exportFlat: exportFlatInput.checked,
    exportDrawing: exportDrawingInput.checked,
    deduplicate: deduplicateInput.checked,
  };

  convertBtn.disabled = true;
  try {
    const bytes = await file.arrayBuffer();
    log("Loading OpenCascade runtime (first visit may take ~20s)...");
    log(`Extracting geometry from ${file.name}...`);
    const geometry = await extractGeometry(bytes, options);
    log(
      `Found ${geometry.flatFaceCount} flat face(s). Writing DXF with ezdxf...`,
    );

    const py = await ensureRuntime();
    py.globals.set("specs_json", JSON.stringify(geometry.specs));

    await py.runPythonAsync(`
import json
import dxf_writer
specs = json.loads(specs_json)
output_files = dxf_writer.write_dxf_from_specs_for_js(specs)
`);

    const outputFiles = py.globals.get("output_files").toJs();
    log(`Done — prepared ${outputFiles.length} DXF file(s).\n`);

    for (const item of outputFiles) {
      const name = item.name;
      const data = Uint8Array.from(item.content);
      const sizeMb = (data.byteLength / (1024 * 1024)).toFixed(2);
      log(`Prepared ${name} (${sizeMb} MB)`);
      downloadBlob(name, data);
    }

    log("\nClick a download link above.");
  } catch (error) {
    log(`Error: ${error.message || error}`);
  } finally {
    convertBtn.disabled = false;
  }
});

window.addEventListener("load", () => {
  ensureRuntime().catch((error) => {
    log(`Runtime preload failed: ${error.message}`);
    log("Click Convert again after fixing your connection.");
  });
});

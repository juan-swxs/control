"use strict";

/* Escalas por defecto; el firmware las confirma con la línea CFG */
const cfg = { sx: 0.4125, sv: 0.1, xmin: -6.4, xmax: 10.5, rmin: -5, rmax: 9, vdd: 3.3, posOk: false, velOk: false, refOk: false, seen: false };
const DESIGN = { ts: 0.39, mp: 1.5 }; // realimentación de estados, tsd = 0.5 s, ζ = 0.8
const HIST_SECONDS = 30;

let port = null, writer = null, reader = null, connected = false;
let lastMsg = 0, sending = false, pendingRef = null;
let hist = [], last = null, dirty = true;
let winSec = 10, paused = false, pausedEnd = 0, cursorT = null;
let rec = null, lastRec = null;

const $ = (id) => document.getElementById(id);
const sl = $("sl"), bubble = $("bubble"), banner = $("banner"), inMm = $("inMm");
const cvPos = $("cvPos"), cvVel = $("cvVel");
const ctxPos = cvPos.getContext("2d"), ctxVel = cvVel.getContext("2d");

/* ============================================================
 * Avisos
 * ============================================================ */
function toast(msg, type = "error") {
  const el = document.createElement("div");
  el.className = `toast ${type}`;
  el.textContent = msg;
  $("toasts").appendChild(el);
  setTimeout(() => el.remove(), 5000);
}
function showBanner(msg) { banner.textContent = msg; banner.classList.remove("hidden"); }
function hideBanner() { banner.classList.add("hidden"); }

if (!("serial" in navigator)) {
  showBanner("Este navegador no soporta Web Serial. Usa Chrome o Edge.");
  $("btnConn").disabled = true;
  $("btnConnEmpty").disabled = true;
}

/* ============================================================
 * Conexión serial
 * ============================================================ */
function setLink(ok) {
  connected = ok;
  if (!ok) cfg.seen = false;
  $("dot").classList.toggle("on", ok);
  $("st").textContent = ok ? "Conectado" : "Desconectado";
  $("btnConn").textContent = ok ? "Desconectar" : "Conectar";
  $("btnConn").classList.toggle("on", ok);
  if (ok) hideBanner();
}

$("btnConn").addEventListener("click", async () => {
  if (connected) return disconnectSerial();
  try {
    port = await navigator.serial.requestPort();
  } catch (e) {
    if (e.name !== "NotFoundError") toast("No se pudo abrir el selector de puertos: " + e.message);
    return;
  }
  try {
    await port.open({ baudRate: 115200 });
    writer = port.writable.getWriter();
    setLink(true);
    toast("Conectado al ESP32", "ok");
    readLoop();
    watchTelemetry();
    setTimeout(() => sendLine("CFG"), 300);
  } catch (e) {
    toast("No se pudo abrir el puerto (¿está en uso por idf.py monitor?): " + e.message);
    port = null;
  }
});

async function disconnectSerial() {
  try {
    if (reader) { await reader.cancel().catch(() => {}); reader = null; }
    if (writer) { writer.releaseLock(); writer = null; }
    if (port) { await port.close(); port = null; }
  } catch (e) { toast("Error al desconectar: " + e.message); }
  setLink(false);
}

$("btnConnEmpty").addEventListener("click", () => $("btnConn").click());

navigator.serial?.addEventListener("disconnect", () => {
  if (connected) { toast("Se desconectó el cable USB"); setLink(false); }
});

function watchTelemetry() {
  const t0 = Date.now();
  const iv = setInterval(() => {
    if (!connected) return clearInterval(iv);
    if (Date.now() - t0 > 3000 && lastMsg < t0) {
      toast("No llega telemetría. Revisa que el firmware esté corriendo y que ningún otro programa tenga el puerto abierto.");
      clearInterval(iv);
    }
  }, 500);
}

setInterval(() => {
  if (connected && lastMsg !== 0 && Date.now() - lastMsg > 1500) {
    lastMsg = 0;
    toast("Se dejó de recibir telemetría del ESP32");
  }
}, 500);

async function readLoop() {
  const decoder = new TextDecoderStream();
  reader = port.readable.pipeThrough(decoder).getReader();
  let buf = "";
  try {
    while (true) {
      const { value, done } = await reader.read();
      if (done) break;
      buf += value;
      const lines = buf.split("\n");
      buf = lines.pop();
      for (const line of lines) handleLine(line.trim());
    }
  } catch (e) {
    if (connected) toast("Se perdió la conexión: " + e.message);
  } finally {
    try { reader.releaseLock(); } catch (_) {}
    setLink(false);
  }
}

async function sendLine(text) {
  if (!connected || !writer) { toast("Conéctate primero."); return false; }
  try { await writer.write(new TextEncoder().encode(text + "\n")); return true; }
  catch (e) { toast("Error enviando comando: " + e.message); return false; }
}

/* ============================================================
 * Líneas del ESP32
 * ============================================================ */
function handleLine(line) {
  if (line.startsWith("DATA,")) return handleData(line);
  if (line.startsWith("CFG,")) return handleCfg(line);
  if (line.startsWith("ERR,")) return toast(line.slice(4));
  if (line.startsWith("ACK,")) return handleAck(line.slice(4).split(","));
  if (line.startsWith("REFST,")) return handleRefSt(line);
}

/* CFG,sx,sv,x_max,r_max,vdd_mcp,pos_ok,vel_ok,ref_ok,...,r_min (18),x_min (19) */
function handleCfg(line) {
  const p = line.split(",").map(Number);
  if (p.length < 8 || p.slice(1).some(Number.isNaN)) return;
  [, cfg.sx, cfg.sv, cfg.xmax, cfg.rmax, cfg.vdd] = p;
  cfg.posOk = p[6] === 1;
  cfg.velOk = p[7] === 1;
  cfg.refOk = p[8] === 1;
  if (p.length > 19) { cfg.rmin = p[18]; cfg.xmin = p[19]; }
  cfg.seen = true;
  paintLimits();
  $("inVdd").value = Math.round(cfg.vdd * 1000);
  paintMcpState();
  paintSlider(parseFloat(sl.value));
}

/* REFST,enlace,mcp_ok,raw,vr_salida_v: lo que de verdad sale del MCP #3 */
const refSt = { link: false, mcp: false, raw: 0, v: 0 };
function handleRefSt(line) {
  const p = line.split(",").map(Number);
  if (p.length < 5 || p.slice(1).some(Number.isNaN)) return;
  refSt.link = p[1] === 1; refSt.mcp = p[2] === 1; refSt.raw = p[3]; refSt.v = p[4];
  const ok = refSt.link && refSt.mcp;
  if (ok !== cfg.refOk) { cfg.refOk = ok; paintMcpState(); }
  dirty = true;
}

function paintMcpState() {
  /* sin la línea CFG todavía no se sabe si MCP #1 y #2 existen: se pide de nuevo */
  if (!cfg.seen) { if (connected) sendLine("CFG"); return; }
  const all = cfg.posOk && cfg.velOk && cfg.refOk;
  const mark = (b) => (b ? "✓" : "✗");
  const pill = $("pillMcp");
  pill.textContent = all ? `MCP #1, #2 y #3 listos, Vdd ${cfg.vdd.toFixed(3)} V`
    : `MCP #1 ${mark(cfg.posOk)}   MCP #2 ${mark(cfg.velOk)}   MCP #3 ${mark(cfg.refOk)}`;
  pill.className = "pill " + (all ? "ok" : "warn");
  const faltan = [];
  if (!cfg.posOk) faltan.push("MCP #1 (P: SDA=GPIO21, SCL=GPIO22)");
  if (!cfg.velOk) faltan.push("MCP #2 (V: SDA=GPIO19, SCL=GPIO18)");
  const msgs = [];
  if (faltan.length) msgs.push(`El ESP32 no encontró ${faltan.join(" ni ")}. Revisa el cableado y la alimentación del módulo.`);
  if (!cfg.refOk) msgs.push(refSt.link
    ? "El ESP32 de referencia responde, pero no encuentra su MCP4725 #3 (SDA=GPIO21, SCL=GPIO22)."
    : "No hay respuesta del ESP32 de referencia por ESP-NOW: revisa que esté encendido. Mientras tanto Vr no cambia.");
  if (msgs.length) showBanner(msgs.join(" ")); else hideBanner();
}

function handleAck(p) {
  const msgs = {
    ZERO: "Posición actual = 0 mm",
    AUTO: "P y V vuelven a seguir al encoder",
    TEST: `Salidas fijas: P = ${p[1]} V, V = ${p[2]} V`,
    VDD: `Vdd de los MCP4725 = ${p[1]} V`,
    REF: `Referencia enviada: ${(+p[1]).toFixed(2)} mm (Vr = ${(+p[2]).toFixed(4)} V)`,
    MODE: `Sentido positivo: ${p[1] === "POS" ? "positivo" : "negativo"} (cero reiniciado)`,
  };
  if (msgs[p[0]]) toast(msgs[p[0]], "ok");
}

/* DATA,t_ms,x_mm,v_mm_s,r_mm,vr_v,p_raw,p_v,vel_raw,vel_v,modo,prueba */
function handleData(line) {
  const p = line.split(",");
  if (p.length < 12) return;
  const n = p.slice(1).map(Number);
  if (n.some(Number.isNaN)) return;
  const [tMs, x, v, r, vr, pRaw, pV, velRaw, velV, mode, test] = n;
  const s = { t: tMs / 1000, x, v, r, vr, pRaw, pV, velRaw, velV, mode, test };

  if (hist.length && s.t < hist[hist.length - 1].t) hist = []; // el ESP32 se reinició
  lastMsg = Date.now();
  last = s;
  hist.push(s);
  while (hist.length && s.t - hist[0].t > HIST_SECONDS) hist.shift();

  if (rec && rec.active) recordSample(s);
  dirty = true;
}

/* ============================================================
 * Tarjetas de valores
 * ============================================================ */
const clamp = (v, a, b) => Math.min(b, Math.max(a, v));
const fmtNum = (v, d, unit) => `${v.toFixed(d)}<span>${unit}</span>`;

function centerBar(el, value, full) {
  const w = clamp(Math.abs(value) / full, 0, 1) * 50;
  el.style.left = `${value >= 0 ? 50 : 50 - w}%`;
  el.style.width = `${w}%`;
}

function renderStats() {
  const s = last;
  if (!s) return;
  const e = s.r - s.x;

  $("x").innerHTML = fmtNum(s.x, 3, "mm");
  const p0 = rackPct(0), px = rackPct(s.x), bx = $("bx");
  bx.style.left = `${Math.min(p0, px)}%`;
  bx.style.width = `${Math.abs(px - p0)}%`;
  bx.classList.toggle("neg", s.x < 0);
  $("mr").style.left = `${rackPct(s.r)}%`;
  $("xfoot").textContent = s.x < 0 ? "x < 0: P se recorta a 0 V" : `${fmtMm(cfg.xmin)} a ${fmtMm(cfg.xmax)} mm`;

  $("v").innerHTML = fmtNum(s.v, 2, "mm/s");
  centerBar($("bv"), s.v, 3.3 / cfg.sv);
  $("vfoot").textContent = s.v < 0 ? "negativa: V se recorta a 0 V" : `V = ${(s.v * cfg.sv).toFixed(3)} V`;

  $("r").innerHTML = fmtNum(s.r, 3, "mm");
  centerBar($("br"), s.r, Math.max(-cfg.rmin, cfg.rmax));
  $("rfoot").textContent = `Vr = ${s.vr.toFixed(4)} V`;

  const eEl = $("e");
  eEl.innerHTML = fmtNum(e, 3, "mm");
  eEl.classList.toggle("ok", Math.abs(e) < 0.1);
  centerBar($("be"), e, cfg.xmax - cfg.xmin);
  $("efoot").textContent = `E = Vr − P = ${(e * cfg.sx).toFixed(3)} V`;

  $("pv").innerHTML = fmtNum(s.pV, 3, "V");
  $("bp").style.width = `${s.pRaw / 4095 * 100}%`;
  $("pfoot").textContent = `raw ${s.pRaw}/4095, ${cfg.sx} V/mm`;

  $("vv").innerHTML = fmtNum(s.velV, 3, "V");
  $("bvv").style.width = `${s.velRaw / 4095 * 100}%`;
  $("vvfoot").textContent = `raw ${s.velRaw}/4095, ${cfg.sv} V/(mm/s)`;

  /* Vr: lo que confirma el ESP32 de referencia, no solo lo pedido */
  const vrOut = cfg.refOk ? refSt.v : s.vr;
  $("vr").innerHTML = fmtNum(vrOut, 3, "V");
  $("bvr").style.width = `${clamp(vrOut / cfg.vdd, 0, 1) * 100}%`;
  $("vrfoot").textContent = cfg.refOk
    ? `raw ${refSt.raw}/4095, pedido ${s.vr.toFixed(3)} V por ESP-NOW`
    : "sin respuesta del ESP32 de referencia";

  $("pillTest").classList.toggle("hidden", s.test !== 1);
}

/* ============================================================
 * Recorrido, límites y cero (el sistema se mueve hacia los dos lados)
 * ============================================================ */
const fmtMm = (v) => (v > 0 ? "+" : v < 0 ? "−" : "") + Math.abs(v).toFixed(Number.isInteger(v) ? 0 : 1);
const rackPct = (mm) => clamp((mm - cfg.xmin) / (cfg.xmax - cfg.xmin), 0, 1) * 100;

function paintLimits() {
  $("limXmin").textContent = fmtMm(cfg.xmin);
  $("limXmax").textContent = fmtMm(cfg.xmax);
  $("limRmin").textContent = fmtMm(cfg.rmin);
  $("limRmax").textContent = fmtMm(cfg.rmax);
  sl.min = inMm.min = cfg.rmin;
  sl.max = inMm.max = cfg.rmax;
  $("rz").style.left = `${rackPct(0)}%`;
  if (!last) { // sin datos todavía: piñón y bandera en el cero
    $("bx").style.left = `${rackPct(0)}%`;
    $("mr").style.left = `${rackPct(0)}%`;
  }
  /* escala de la cremallera: un número por milímetro entero, posición real */
  const rs = $("rackScale");
  rs.innerHTML = "";
  for (let mm = Math.ceil(cfg.xmin); mm <= Math.floor(cfg.xmax); mm++) {
    if (mm % 2 && mm !== Math.ceil(cfg.xmin) && mm !== Math.floor(cfg.xmax)) continue;
    const s = document.createElement("span");
    s.textContent = mm === 0 ? "0" : fmtMm(mm);
    s.style.left = `${rackPct(mm)}%`;
    if (mm === 0) s.className = "zero";
    rs.appendChild(s);
  }
  const lab = document.createElement("span");
  lab.textContent = "mm"; lab.className = "unit"; rs.appendChild(lab);
  /* escala del slider */
  const ss = $("slScale");
  ss.innerHTML = "";
  [cfg.rmin, cfg.rmin / 2, 0, cfg.rmax / 2, cfg.rmax].forEach((v) => {
    const s = document.createElement("span");
    s.textContent = v === 0 ? "0" : fmtMm(+v.toFixed(1));
    ss.appendChild(s);
  });
  document.querySelectorAll(".presets button[data-lim]").forEach((b) => {
    const v = b.dataset.lim === "min" ? cfg.rmin : cfg.rmax;
    b.dataset.mm = v;
    b.firstChild.textContent = `${fmtMm(v)} mm`;
  });
  paintSlider(parseFloat(sl.value));
}

$("btnZero").addEventListener("click", () => { sendLine("ZERO"); hist = []; });

/* ============================================================
 * Referencia
 * ============================================================ */
function paintSlider(mm) {
  const pct = (mm - cfg.rmin) / (cfg.rmax - cfg.rmin) * 100;
  sl.style.setProperty("--p", `${pct.toFixed(1)}%`);
  $("sv").innerHTML = fmtNum(mm, 2, "mm");
  $("svv").textContent = `Vr ${clamp(mm * cfg.sx, 0, cfg.vdd).toFixed(4)} V`;
  bubble.textContent = `${mm.toFixed(2)} mm`;
  const thumb = 26, trackWidth = sl.clientWidth - thumb;
  bubble.style.left = `${thumb / 2 + (pct / 100) * trackWidth}px`;
  $("btnStep").textContent = `Enviar escalón a ${mm.toFixed(2)} mm y grabar`;
}

function setTarget(mm) {
  mm = clamp(mm, cfg.rmin, cfg.rmax);
  sl.value = mm;
  inMm.value = mm.toFixed(2);
  paintSlider(mm);
  return mm;
}

/* Envía REF sin saturar el puerto: si hay uno en camino, guarda solo el último */
function sendRef(mm) {
  if (sending) { pendingRef = mm; return; }
  sending = true;
  sendLine(`REF ${mm.toFixed(3)}`).finally(() => {
    sending = false;
    if (pendingRef !== null) { const r = pendingRef; pendingRef = null; sendRef(r); }
  });
}

sl.addEventListener("input", () => setTarget(parseFloat(sl.value)));
inMm.addEventListener("input", () => {
  const mm = parseFloat(inMm.value);
  if (!Number.isNaN(mm)) { sl.value = clamp(mm, cfg.rmin, cfg.rmax); paintSlider(parseFloat(sl.value)); }
});
inMm.addEventListener("keydown", (e) => { if (e.key === "Enter") $("btnApply").click(); });
$("btnApply").addEventListener("click", () => {
  const mm = parseFloat(inMm.value);
  if (Number.isNaN(mm)) return toast("Escribe una referencia en mm.");
  if (mm > cfg.rmax || mm < cfg.rmin)
    toast(`La referencia va de ${fmtMm(cfg.rmin)} a ${fmtMm(cfg.rmax)} mm (el recorrido es ${fmtMm(cfg.xmin)} a ${fmtMm(cfg.xmax)} mm): se recorta.`);
  sendRef(setTarget(mm));
});
document.querySelectorAll(".presets button").forEach((b) =>
  b.addEventListener("click", () => sendRef(setTarget(parseFloat(b.dataset.mm))))
);
window.addEventListener("resize", () => paintSlider(parseFloat(sl.value)));

/* ============================================================
 * Prueba de escalón: grabación y métricas
 * ============================================================ */
function startRecording(label) {
  if (!connected) { toast("Conéctate primero."); return false; }
  rec = { active: true, dur: parseFloat($("selDur").value), samples: [], t0: null, label, started: Date.now() };
  $("btnRec").textContent = "■ Detener";
  $("btnRec").classList.add("rec");
  $("btnCsv").disabled = true;
  $("recProg").classList.remove("hidden");
  $("recBar").style.width = "0%";
  ["mTs", "mMp", "mTr", "mEss", "mVmax"].forEach((id) => { $(id).textContent = "…"; $(id).className = ""; });
  $("recInfo").textContent = "Grabando…";
  return true;
}

function recordSample(s) {
  if (rec.t0 === null) rec.t0 = s.t;
  rec.samples.push(s);
  const el = s.t - rec.t0;
  $("recBar").style.width = `${clamp(el / rec.dur, 0, 1) * 100}%`;
  if (el >= rec.dur) stopRecording();
}

function stopRecording() {
  if (!rec) return;
  rec.active = false;
  rec.tEnd = rec.samples.length ? rec.samples[rec.samples.length - 1].t : null;
  lastRec = rec;
  $("btnRec").textContent = "● Grabar";
  $("btnRec").classList.remove("rec");
  $("recProg").classList.add("hidden");
  $("btnCsv").disabled = rec.samples.length < 2;
  showMetrics(analyzeStep(rec.samples));
}

$("btnRec").addEventListener("click", () => {
  if (rec && rec.active) return stopRecording();
  startRecording("manual");
});

/* Primero empieza a grabar y después manda r: así el escalón queda dentro
 * de la grabación y t₀ es el instante en que cambia la referencia */
$("btnStep").addEventListener("click", () => {
  if (rec && rec.active) return toast("Ya hay una grabación en curso.");
  if (!connected) return toast("Conéctate primero.");
  if (!startRecording("escalon")) return;
  setTimeout(() => {
    sendRef(parseFloat(sl.value));
    toast(`Escalón a ${parseFloat(sl.value).toFixed(2)} mm enviado, grabando`, "ok");
  }, 200);
});

/* Ubica el escalón (cambio de r, o inicio de movimiento si se dio abriendo S1) */
function analyzeStep(s) {
  if (s.length < 10) return { error: "Muy pocas muestras" };
  const r0 = s[0].r;
  let k0 = s.findIndex((p) => Math.abs(p.r - r0) > 0.01);
  let how = "t₀ = cambio de referencia";
  let x0;
  if (k0 >= 0) {
    x0 = s[k0].x;
  } else {
    x0 = s[0].x;
    k0 = s.findIndex((p) => Math.abs(p.x - x0) > 0.1);
    if (k0 < 0) return { error: "La masa no se movió durante la grabación" };
    if (s[s.length - 1].r < 0.05) return { error: "Registra la referencia (el valor de Vr en mm) antes de grabar" };
    k0 = Math.max(0, k0 - 1);
    how = "t₀ = inicio del movimiento (escalón por S1)";
  }
  const t0 = s[k0].t, rf = s[s.length - 1].r, A = rf - x0;
  if (Math.abs(A) < 0.2) return { error: "El escalón es muy pequeño (< 0.2 mm)" };

  const post = s.slice(k0), sg = Math.sign(A), absA = Math.abs(A);
  const peak = Math.max(...post.map((p) => sg * (p.x - rf)));
  const mp = Math.max(0, peak) / absA * 100;

  const band = 0.02 * absA;
  let lastOut = -1;
  post.forEach((p, i) => { if (Math.abs(p.x - rf) > band) lastOut = i; });
  const ts = lastOut === post.length - 1 ? null : (lastOut < 0 ? 0 : post[lastOut + 1].t - t0);

  const t10 = post.find((p) => sg * (p.x - x0) >= 0.1 * absA);
  const t90 = post.find((p) => sg * (p.x - x0) >= 0.9 * absA);
  const tr = t10 && t90 ? t90.t - t10.t : null;

  const nTail = Math.max(5, Math.round(post.length * 0.1));
  const tail = post.slice(-nTail);
  const ess = rf - tail.reduce((a, p) => a + p.x, 0) / tail.length;
  const vmax = Math.max(...post.map((p) => Math.abs(p.v)));

  return { ts, mp, tr, ess, vmax, how, A, rf, t0 };
}

function setMetric(id, text, cls = "") { $(id).textContent = text; $(id).className = cls; }

function showMetrics(m) {
  if (m.error) {
    ["mTs", "mMp", "mTr", "mEss", "mVmax"].forEach((id) => setMetric(id, "—"));
    $("recInfo").textContent = m.error;
    return;
  }
  if (lastRec) lastRec.tStep = m.t0;
  setMetric("mTs", m.ts === null ? "no se estableció" : `${m.ts.toFixed(2)} s`,
    m.ts !== null && Math.abs(m.ts - DESIGN.ts) <= 0.3 * DESIGN.ts ? "good" : "badv");
  setMetric("mMp", `${m.mp.toFixed(1)} %`, m.mp <= 5 ? "good" : "badv");
  setMetric("mTr", m.tr === null ? "—" : `${m.tr.toFixed(2)} s`);
  setMetric("mEss", `${m.ess.toFixed(3)} mm`, Math.abs(m.ess) <= 0.1 ? "good" : "badv");
  setMetric("mVmax", `${m.vmax.toFixed(1)} mm/s`);
  $("recInfo").textContent = `Escalón de ${m.A.toFixed(2)} mm hacia ${m.rf.toFixed(2)} mm, ${m.how}. Banda del 2 % = ±${(0.02 * Math.abs(m.A)).toFixed(2)} mm (1 cuenta del encoder ≈ 0.067 mm).`;
}

$("btnCsv").addEventListener("click", () => {
  if (!lastRec || lastRec.samples.length < 2) return;
  const t0 = lastRec.tStep ?? lastRec.samples[0].t;
  const rows = ["t_s,r_mm,x_mm,v_mm_s,Vr_V,P_V,V_V"];
  for (const p of lastRec.samples) {
    rows.push([(p.t - t0).toFixed(3), p.r.toFixed(3), p.x.toFixed(3), p.v.toFixed(2),
      p.vr.toFixed(4), p.pV.toFixed(3), p.velV.toFixed(3)].join(","));
  }
  const rf = lastRec.samples[lastRec.samples.length - 1].r;
  const stamp = new Date().toISOString().slice(0, 19).replace(/[:T]/g, "-");
  const a = document.createElement("a");
  a.href = URL.createObjectURL(new Blob([rows.join("\n")], { type: "text/csv" }));
  a.download = `lab4_escalon_${rf.toFixed(2)}mm_${stamp}.csv`;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
});

/* ============================================================
 * Pruebas de banco y calibración
 * ============================================================ */
function sendTest() {
  const p = clamp(parseInt($("inP").value, 10) || 0, 0, 3300);
  const v = clamp(parseInt($("inV").value, 10) || 0, 0, 3300);
  sendLine(`TEST ${p} ${v}`);
}
$("btnTest").addEventListener("click", sendTest);
$("btnAuto").addEventListener("click", () => sendLine("AUTO"));
document.querySelectorAll(".bench-presets button").forEach((b) =>
  b.addEventListener("click", () => { $("inP").value = b.dataset.p; $("inV").value = b.dataset.v; sendTest(); })
);
$("btnVdd").addEventListener("click", () => {
  const mv = parseInt($("inVdd").value, 10);
  if (!(mv >= 2700 && mv <= 5500)) return toast("Vdd debe estar entre 2700 y 5500 mV.");
  sendLine(`VDD ${mv}`);
});

/* ============================================================
 * Gráficas
 * ============================================================ */
/* Mismos colores que la cremallera: x cian, r violeta, ẋ lavanda, sobre la pantalla azul */
const COL = {
  grid: "rgba(226,234,255,.08)", zero: "rgba(226,234,255,.24)", label: "rgba(226,234,255,.6)",
  pos: "#4FD2E9", ref: "#CF7DFF", vel: "#9DA8FF",
  rec: "rgba(181,65,250,.10)", cursor: "rgba(255,255,255,.45)",
  err: "rgba(207,125,255,.22)", band: "rgba(207,125,255,.07)", bandEdge: "rgba(207,125,255,.45)",
  tagInk: "#0F1756",
};
const MONO = "500 12.5px 'Barlow Semi Condensed', 'Segoe UI', system-ui, sans-serif";
const TAG_FONT = "600 12.5px 'Barlow Semi Condensed', 'Segoe UI', system-ui, sans-serif";
const PAD = { l: 40, r: 66 }; // a la derecha van las etiquetas con el valor actual

function fitCanvas(cv, ctx) {
  const k = window.devicePixelRatio || 1;
  cv.width = cv.clientWidth * k; cv.height = cv.clientHeight * k;
  ctx.setTransform(k, 0, 0, k, 0, 0);
  dirty = true;
}
new ResizeObserver(() => { fitCanvas(cvPos, ctxPos); fitCanvas(cvVel, ctxVel); }).observe(cvPos);

function niceStep(range, target) {
  const raw = range / target, mag = 10 ** Math.floor(Math.log10(raw));
  for (const m of [1, 2, 2.5, 5, 10]) if (m * mag >= raw) return m * mag;
  return 10 * mag;
}

function timeWindow() {
  const tEnd = paused ? pausedEnd : (hist.length ? hist[hist.length - 1].t : 0);
  return { t1: tEnd, t0: tEnd - winSec };
}

function drawAxes(ctx, W, H, top, bottom, lo, hi, t0, t1, unit, showTime) {
  const py = (v) => top + (hi - v) / (hi - lo) * (H - top - bottom);
  const px = (t) => PAD.l + (t - t0) / (t1 - t0) * (W - PAD.l - PAD.r);
  ctx.font = MONO;
  ctx.lineWidth = 1;

  /* solo líneas horizontales: el tiempo se lee en el eje de abajo */
  const ys = niceStep(hi - lo, 5);
  for (let v = Math.ceil(lo / ys) * ys; v <= hi + 1e-9; v += ys) {
    const y = Math.round(py(v)) + 0.5;
    ctx.strokeStyle = Math.abs(v) < 1e-9 ? COL.zero : COL.grid;
    ctx.beginPath(); ctx.moveTo(PAD.l, y); ctx.lineTo(W - PAD.r, y); ctx.stroke();
    ctx.fillStyle = COL.label; ctx.textAlign = "right";
    ctx.fillText(+v.toFixed(2), PAD.l - 8, y + 4);
  }
  ctx.textAlign = "left";
  ctx.fillText(unit, PAD.l + 4, top + 12);

  if (showTime) {
    const ts = winSec <= 5 ? 1 : winSec <= 10 ? 2 : 5;
    ctx.fillStyle = COL.label;
    /* marcas relativas a "ahora" (borde derecho), cada ts segundos hacia atrás */
    ctx.textAlign = "center";
    for (let k = ts; k <= winSec + 1e-9; k += ts) {
      ctx.fillText(`−${k} s`, px(t1 - k), H - 4);
    }
    ctx.textAlign = "right";
    ctx.fillText("ahora", px(t1), H - 4);
  }
  return { px, py };
}

function tracePath(ctx, pts, key, px, py) {
  pts.forEach((p, i) => (i ? ctx.lineTo(px(p.t), py(p[key])) : ctx.moveTo(px(p.t), py(p[key]))));
}

function drawLine(ctx, pts, key, px, py, color, { dash = [], width = 2.2 } = {}) {
  if (pts.length < 2) return;
  ctx.save();
  ctx.setLineDash(dash);
  ctx.strokeStyle = color; ctx.lineWidth = width; ctx.lineJoin = "round"; ctx.lineCap = "round";
  ctx.beginPath(); tracePath(ctx, pts, key, px, py); ctx.stroke();
  ctx.restore();
}

/* Área entre r y x: es el error que el controlador tiene que llevar a cero */
function drawErrorBand(ctx, pts, px, py) {
  if (pts.length < 2) return;
  ctx.beginPath();
  tracePath(ctx, pts, "x", px, py);
  for (let i = pts.length - 1; i >= 0; i--) ctx.lineTo(px(pts[i].t), py(pts[i].r));
  ctx.closePath();
  ctx.fillStyle = COL.err; ctx.fill();
}

/* Banda de ±2 % alrededor de la referencia actual (criterio de t_s) */
function drawSettlingBand(ctx, r, W, px0, py) {
  if (Math.abs(r) < 0.2) return;
  const d = 0.02 * Math.abs(r), yA = py(r + d), yB = py(r - d);
  ctx.fillStyle = COL.band;
  ctx.fillRect(px0, yA, W - PAD.r - px0, yB - yA);
  ctx.save();
  ctx.setLineDash([3, 4]); ctx.strokeStyle = COL.bandEdge; ctx.lineWidth = 1;
  for (const y of [yA, yB]) { ctx.beginPath(); ctx.moveTo(px0, Math.round(y) + .5); ctx.lineTo(W - PAD.r, Math.round(y) + .5); ctx.stroke(); }
  ctx.restore();
}

/* Velocidad: relleno por encima y por debajo de cero con el mismo color */
function drawSignedArea(ctx, pts, key, px, py, color) {
  if (pts.length < 2) return;
  const y0 = py(0);
  ctx.beginPath();
  tracePath(ctx, pts, key, px, py);
  ctx.lineTo(px(pts[pts.length - 1].t), y0); ctx.lineTo(px(pts[0].t), y0); ctx.closePath();
  ctx.fillStyle = color + "38"; ctx.fill();
}

/* Etiqueta con el valor actual pegada al borde derecho, a la altura de la serie */
function valueTag(ctx, W, y, text, bg, ink, H) {
  ctx.font = TAG_FONT;
  const w = ctx.measureText(text).width + 14, h = 20;
  const x = W - PAD.r + 8, yy = clamp(y - h / 2, 2, H - h - 2);
  ctx.fillStyle = bg;
  ctx.beginPath(); ctx.roundRect(x, yy, w, h, 5); ctx.fill();
  ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x - 5, y); ctx.strokeStyle = bg; ctx.lineWidth = 1.5; ctx.stroke();
  ctx.fillStyle = ink; ctx.textAlign = "left";
  ctx.fillText(text, x + 7, yy + 14);
}

/* Dos etiquetas no deben encimarse: la de abajo se corre */
function placeTags(a, b, min = 22) {
  if (Math.abs(a.y - b.y) >= min) return;
  const mid = (a.y + b.y) / 2, up = a.y <= b.y ? a : b, down = up === a ? b : a;
  up.y = mid - min / 2; down.y = mid + min / 2;
}

function shadeRecording(ctx, px, top, H, bottom, t0, t1) {
  const r = rec && rec.active ? rec : lastRec;
  if (!r || r.t0 === null) return;
  const a = Math.max(r.t0, t0), b = Math.min(r.active ? t1 : (r.tEnd ?? t1), t1);
  if (b <= a) return;
  ctx.fillStyle = COL.rec;
  ctx.fillRect(px(a), top, px(b) - px(a), H - top - bottom);
}

/* ---------- Estado vacío: respuesta de diseño dibujándose una vez ----------
 * Segundo orden con Mp = 1.5 % y ts(2 %) = 0.39 s (diseño por realimentación de estados) */
const GHOST = (() => {
  const lnM = Math.log(DESIGN.mp / 100);
  const z = -lnM / Math.sqrt(Math.PI ** 2 + lnM ** 2);
  return { z, wn: 4 / (z * DESIGN.ts), step: 5 };
})();
const reduceMotion = window.matchMedia?.("(prefers-reduced-motion: reduce)").matches;
let ghostT0 = null;

function ghostSample(ts) { // ts: tiempo desde el escalón
  if (ts <= 0) return { x: 0, v: 0 };
  const { z, wn, step } = GHOST, wd = wn * Math.sqrt(1 - z * z), e = Math.exp(-z * wn * ts);
  return {
    x: step * (1 - e * (Math.cos(wd * ts) + z / Math.sqrt(1 - z * z) * Math.sin(wd * ts))),
    v: step * wn / Math.sqrt(1 - z * z) * e * Math.sin(wd * ts),
  };
}

function drawGhost() {
  if (ghostT0 === null) ghostT0 = performance.now();
  const raw = reduceMotion ? 1 : clamp((performance.now() - ghostT0) / 1800, 0, 1);
  const prog = 1 - (1 - raw) ** 3;
  if (raw < 1) dirty = true; // sigue animando hasta terminar el trazo

  const tStep = winSec * 0.1, N = 260;
  const pts = [];
  for (let i = 0; i <= N; i++) {
    const t = winSec * i / N, g = ghostSample(t - tStep);
    pts.push({ t, x: g.x, v: g.v, r: t < tStep ? 0 : GHOST.step });
  }

  /* posición */
  let W = cvPos.clientWidth, H = cvPos.clientHeight;
  ctxPos.clearRect(0, 0, W, H);
  const A = drawAxes(ctxPos, W, H, 8, 8, cfg.rmin - 0.5, cfg.rmax + 0.5, 0, winSec, "mm", false);
  const xEnd = A.px(winSec * prog);
  ctxPos.save();
  ctxPos.beginPath(); ctxPos.rect(0, 0, xEnd, H); ctxPos.clip();
  drawSettlingBand(ctxPos, GHOST.step, W, A.px(tStep), A.py);
  drawErrorBand(ctxPos, pts, A.px, A.py);
  /* vidrio bajo la curva */
  const g = ctxPos.createLinearGradient(0, A.py(GHOST.step), 0, A.py(0));
  g.addColorStop(0, "rgba(79,210,233,.28)"); g.addColorStop(1, "rgba(79,210,233,0)");
  ctxPos.beginPath(); tracePath(ctxPos, pts, "x", A.px, A.py);
  ctxPos.lineTo(A.px(winSec), A.py(0)); ctxPos.lineTo(A.px(0), A.py(0)); ctxPos.closePath();
  ctxPos.fillStyle = g; ctxPos.fill();
  drawLine(ctxPos, pts, "r", A.px, A.py, COL.ref, { dash: [7, 5], width: 1.8 });
  ctxPos.shadowColor = "rgba(79,210,233,.8)"; ctxPos.shadowBlur = 14;
  drawLine(ctxPos, pts, "x", A.px, A.py, COL.pos, { width: 3 });
  ctxPos.restore();
  /* punta luminosa mientras se dibuja */
  if (raw < 1) {
    const tp = winSec * prog, gp = ghostSample(tp - tStep);
    ctxPos.beginPath(); ctxPos.arc(A.px(tp), A.py(gp.x), 9, 0, 6.3); ctxPos.fillStyle = "rgba(79,210,233,.25)"; ctxPos.fill();
    ctxPos.beginPath(); ctxPos.arc(A.px(tp), A.py(gp.x), 4, 0, 6.3); ctxPos.fillStyle = "#FFFFFF"; ctxPos.fill();
  }

  /* velocidad */
  W = cvVel.clientWidth; H = cvVel.clientHeight;
  ctxVel.clearRect(0, 0, W, H);
  const vm = Math.max(10, ...pts.map((p) => Math.abs(p.v) * 1.15));
  const B = drawAxes(ctxVel, W, H, 8, 20, -vm, vm, 0, winSec, "mm/s", false);
  ctxVel.save();
  ctxVel.beginPath(); ctxVel.rect(0, 0, B.px(winSec * prog), H); ctxVel.clip();
  drawSignedArea(ctxVel, pts, "v", B.px, B.py, COL.vel);
  drawLine(ctxVel, pts, "v", B.px, B.py, COL.vel, { width: 2 });
  ctxVel.restore();
}

function drawCharts() {
  const { t0, t1 } = timeWindow();
  const pts = hist.filter((p) => p.t >= t0 - 0.1);

  /* sin telemetría: respuesta de diseño + invitación a conectar */
  const empty = hist.length === 0;
  $("chartEmpty").classList.toggle("hidden", !empty);
  if (empty) { drawGhost(); $("cursorRead").classList.add("hidden"); return; }
  ghostT0 = null;

  /* posición y referencia */
  let W = cvPos.clientWidth, H = cvPos.clientHeight;
  ctxPos.clearRect(0, 0, W, H);
  let lo = Math.min(-0.5, ...pts.map((p) => p.r)) - 0.3, hi = 0.5;
  for (const p of pts) { lo = Math.min(lo, p.x - 0.3); hi = Math.max(hi, p.x + 0.3, p.r + 0.3); }
  const A = drawAxes(ctxPos, W, H, 8, 8, lo, hi, t0, t1, "mm", false);
  shadeRecording(ctxPos, A.px, 8, H, 8, t0, t1);
  const lastP = pts[pts.length - 1];
  if (lastP) drawSettlingBand(ctxPos, lastP.r, W, PAD.l, A.py);
  drawErrorBand(ctxPos, pts, A.px, A.py);
  drawLine(ctxPos, pts, "r", A.px, A.py, COL.ref, { dash: [7, 5], width: 1.8 });
  drawLine(ctxPos, pts, "x", A.px, A.py, COL.pos, { width: 2.6 });
  if (lastP) {
    const cx = A.px(lastP.t), cy = A.py(lastP.x);
    ctxPos.beginPath(); ctxPos.arc(cx, cy, 7, 0, 6.3); ctxPos.fillStyle = COL.pos + "40"; ctxPos.fill();
    ctxPos.beginPath(); ctxPos.arc(cx, cy, 3.5, 0, 6.3); ctxPos.fillStyle = COL.pos; ctxPos.fill();
    const tx = { y: cy }, tr = { y: A.py(lastP.r) };
    placeTags(tx, tr);
    valueTag(ctxPos, W, tr.y, `r ${lastP.r.toFixed(2)}`, COL.ref, COL.tagInk, H);
    valueTag(ctxPos, W, tx.y, `x ${lastP.x.toFixed(2)}`, COL.pos, COL.tagInk, H);
  }

  /* velocidad */
  W = cvVel.clientWidth; H = cvVel.clientHeight;
  ctxVel.clearRect(0, 0, W, H);
  let vm = 10;
  for (const p of pts) vm = Math.max(vm, Math.abs(p.v) * 1.15);
  const B = drawAxes(ctxVel, W, H, 8, 20, -vm, vm, t0, t1, "mm/s", true);
  shadeRecording(ctxVel, B.px, 8, H, 20, t0, t1);
  drawSignedArea(ctxVel, pts, "v", B.px, B.py, COL.vel);
  drawLine(ctxVel, pts, "v", B.px, B.py, COL.vel, { width: 2 });
  if (lastP) valueTag(ctxVel, W, B.py(lastP.v), `ẋ ${lastP.v.toFixed(1)}`, COL.vel, COL.tagInk, H - 14);

  /* leyenda con el último valor */
  if (lastP) {
    $("lgR").textContent = `${lastP.r.toFixed(2)} mm`;
    $("lgX").textContent = `${lastP.x.toFixed(2)} mm`;
    $("lgV").textContent = `${lastP.v.toFixed(1)} mm/s`;
  }

  /* cursor: línea vertical y un punto en cada serie */
  const read = $("cursorRead");
  if (cursorT === null || !pts.length) { read.classList.add("hidden"); return; }
  let best = pts[0];
  for (const p of pts) if (Math.abs(p.t - cursorT) < Math.abs(best.t - cursorT)) best = p;
  for (const [ctx, P, h, b] of [[ctxPos, A, cvPos.clientHeight, 8], [ctxVel, B, cvVel.clientHeight, 20]]) {
    const x = Math.round(P.px(best.t)) + 0.5;
    ctx.save(); ctx.setLineDash([2, 3]);
    ctx.strokeStyle = COL.cursor; ctx.beginPath(); ctx.moveTo(x, 8); ctx.lineTo(x, h - b); ctx.stroke();
    ctx.restore();
  }
  const dot = (ctx, x, y, c) => {
    ctx.beginPath(); ctx.arc(x, y, 4.5, 0, 6.3); ctx.fillStyle = c; ctx.fill();
    ctx.lineWidth = 2; ctx.strokeStyle = "#0F1756"; ctx.stroke();
  };
  dot(ctxPos, A.px(best.t), A.py(best.r), COL.ref);
  dot(ctxPos, A.px(best.t), A.py(best.x), COL.pos);
  dot(ctxVel, B.px(best.t), B.py(best.v), COL.vel);
  read.innerHTML = `<b>${(best.t - t1).toFixed(2)} s</b><br>r ${best.r.toFixed(3)} mm<br>x ${best.x.toFixed(3)} mm<br>e ${(best.r - best.x).toFixed(3)} mm<br>ẋ ${best.v.toFixed(2)} mm/s`;
  read.classList.remove("hidden");
}

for (const cv of [cvPos, cvVel]) {
  cv.addEventListener("pointermove", (e) => {
    const { t0, t1 } = timeWindow();
    const rect = cv.getBoundingClientRect();
    const f = (e.clientX - rect.left - PAD.l) / (rect.width - PAD.l - PAD.r);
    cursorT = f < 0 || f > 1 ? null : t0 + f * (t1 - t0);
    dirty = true;
  });
  cv.addEventListener("pointerleave", () => { cursorT = null; dirty = true; });
}

document.querySelectorAll("#segWin button").forEach((b) =>
  b.addEventListener("click", () => {
    document.querySelectorAll("#segWin button").forEach((o) => o.classList.toggle("active", o === b));
    winSec = parseInt(b.dataset.win, 10);
    dirty = true;
  })
);
$("btnPause").addEventListener("click", () => {
  paused = !paused;
  pausedEnd = hist.length ? hist[hist.length - 1].t : 0;
  $("btnPause").textContent = paused ? "▶" : "❚❚";
  $("btnPause").classList.toggle("on", paused);
  $("btnPause").title = paused ? "Reanudar gráfica" : "Pausar gráfica";
  dirty = true;
});

/* ============================================================
 * Bucle de dibujo
 * ============================================================ */
function frame() {
  if (dirty) {
    dirty = false;
    renderStats();
    drawCharts();
  }
  requestAnimationFrame(frame);
}

setTarget(0);
document.fonts?.ready.then(() => { paintSlider(parseFloat(sl.value)); dirty = true; });
requestAnimationFrame(frame);

paintLimits();

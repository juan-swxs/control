"use strict";

const VREF = 3.3, MAX_POINTS = 300;
let xmax = 8, hist = [];
let port = null, writer = null, reader = null, connected = false;
let lastMsg = 0, sending = false, pending = null, userTouching = false;

const $ = (id) => document.getElementById(id);
const sl = $("sl"), cv = $("cv"), ctx = cv.getContext("2d");
const banner = $("banner"), bubble = $("bubble");

function toast(msg, type = "error") {
  const el = document.createElement("div");
  el.className = `toast ${type}`;
  el.textContent = msg;
  $("toasts").appendChild(el);
  setTimeout(() => el.remove(), 5000);
}
function showBanner(msg) { banner.textContent = msg; banner.className = "banner"; }
function hideBanner() { banner.className = "banner hidden"; }

if (!("serial" in navigator)) {
  showBanner("Este navegador no soporta Web Serial. Usa Chrome o Edge.");
  $("btnConn").disabled = true;
}

function setLink(ok) {
  connected = ok;
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

/* ---- selector de modo (barra superior) ---- */
const toggleEl = $("toggle"), optPos = $("optPos"), optNeg = $("optNeg");

function setModeUI(mode) { // 0 = POS, 1 = NEG
  toggleEl.classList.toggle("neg", mode === 1);
  optPos.classList.toggle("active", mode === 0);
  optNeg.classList.toggle("active", mode === 1);
}

optPos.addEventListener("click", () => { sendLine("MODE POS"); setModeUI(0); });
optNeg.addEventListener("click", () => { sendLine("MODE NEG"); setModeUI(1); });

/* Formato: DATA,t_ms,x_mm,vel,dac_auto_raw,dac_auto_v,dac_man_raw,dac_man_v,modo,xmax */
function handleLine(line) {
  if (!line.startsWith("DATA,")) return;
  const p = line.split(",");
  if (p.length < 10) return;

  const x = parseFloat(p[2]), vel = parseFloat(p[3]);
  const autoRaw = parseInt(p[4], 10), autoV = parseFloat(p[5]);
  const manRaw = parseInt(p[6], 10), manV = parseFloat(p[7]);
  const mode = parseInt(p[8], 10);
  const xmaxRecv = parseFloat(p[9]);

  if ([x, vel, autoRaw, autoV, manRaw, manV, mode, xmaxRecv].some(Number.isNaN)) return;

  lastMsg = Date.now();
  xmax = xmaxRecv;
  setModeUI(mode);

  $("x").innerHTML = `${x.toFixed(3)}<span>mm</span>`;
  $("vel").textContent = `${vel.toFixed(2)} mm/s`;
  $("bx").style.width = `${Math.min(100, Math.abs(x) / xmax * 100)}%`;

  $("va").innerHTML = `${autoV.toFixed(3)}<span>V</span>`;
  $("ba").style.width = `${autoRaw / 255 * 100}%`;
  $("ra").textContent = `raw ${autoRaw}/255`;

  $("vm").innerHTML = `${manV.toFixed(3)}<span>V</span>`;
  $("bm").style.width = `${manRaw / 255 * 100}%`;
  $("rm").textContent = `raw ${manRaw}/255`;

  if (!userTouching && parseInt(sl.value, 10) !== manRaw) {
    sl.value = manRaw; paintSlider(manRaw);
  }

  hist.push(x);
  if (hist.length > MAX_POINTS) hist.shift();
  draw();
}

/* ---- slider con burbuja flotante ---- */
function paintSlider(raw) {
  const pct = raw / 255 * 100;
  sl.style.setProperty("--p", `${pct.toFixed(1)}%`);
  $("sv").innerHTML = `${(raw * VREF / 255).toFixed(3)}<span>V</span>`;
  $("sraw").textContent = `raw ${raw}/255`;

  bubble.textContent = `${(raw * VREF / 255).toFixed(2)} V`;
  const trackWidth = sl.clientWidth - 26;
  const left = 13 + (pct / 100) * trackWidth;
  bubble.style.left = `${left}px`;
}

async function sendLine(text) {
  if (!connected || !writer) return toast("Conéctate primero.");
  try { await writer.write(new TextEncoder().encode(text + "\n")); }
  catch (e) { toast("Error enviando comando: " + e.message); }
}

function queueSend(raw) {
  if (sending) { pending = raw; return; }
  sending = true;
  sendLine("RAW " + raw).finally(() => {
    sending = false;
    if (pending !== null) { const r = pending; pending = null; queueSend(r); }
  });
}

sl.addEventListener("input", () => {
  userTouching = true;
  const raw = parseInt(sl.value, 10);
  paintSlider(raw); queueSend(raw);
});
["change", "mouseup", "touchend", "pointerup"].forEach((ev) =>
  sl.addEventListener(ev, () => setTimeout(() => { userTouching = false; }, 400))
);
window.addEventListener("resize", () => paintSlider(parseInt(sl.value, 10)));

document.querySelectorAll(".btns button[data-mv]").forEach((b) => {
  b.addEventListener("click", () => {
    const raw = Math.round(parseInt(b.dataset.mv, 10) / 3300 * 255);
    sl.value = raw; paintSlider(raw); queueSend(raw);
  });
});
$("bz").addEventListener("click", () => { sendLine("ZERO"); hist = []; });

/* ---- gráfica ---- */
function resize() {
  const r = window.devicePixelRatio || 1;
  cv.width = cv.clientWidth * r; cv.height = cv.clientHeight * r;
  ctx.setTransform(r, 0, 0, r, 0, 0);
}
new ResizeObserver(resize).observe(cv);
resize();

function draw() {
  const W = cv.clientWidth, H = cv.clientHeight, pad = 8;
  ctx.clearRect(0, 0, W, H);

  let lim = xmax;
  for (const v of hist) lim = Math.max(lim, Math.abs(v) * 1.15);

  ctx.strokeStyle = "rgba(255,255,255,.08)"; ctx.lineWidth = 1;
  ctx.fillStyle = "rgba(255,255,255,.4)"; ctx.font = "10px -apple-system,sans-serif";
  for (let k = -1; k <= 1; k++) {
    const y = H / 2 - k * (H / 2 - pad);
    ctx.beginPath(); ctx.moveTo(28, y); ctx.lineTo(W, y); ctx.stroke();
    ctx.fillText((k * lim).toFixed(1), 0, y + 3);
  }
  if (hist.length < 2) return;

  const step = (W - 28) / (MAX_POINTS - 1);
  const px = (i) => 28 + i * step;
  const py = (v) => H / 2 - (v / lim) * (H / 2 - pad);

  ctx.beginPath();
  hist.forEach((v, i) => (i ? ctx.lineTo(px(i), py(v)) : ctx.moveTo(px(i), py(v))));
  ctx.strokeStyle = "#d9a860"; ctx.lineWidth = 1.8; ctx.lineJoin = "round"; ctx.stroke();

  const lx = px(hist.length - 1), ly = py(hist[hist.length - 1]);
  ctx.beginPath(); ctx.arc(lx, ly, 3.5, 0, 6.3); ctx.fillStyle = "#d9a860"; ctx.fill();
}

setInterval(() => {
  if (connected && lastMsg !== 0 && Date.now() - lastMsg > 1500) {
    setLink(false);
    toast("Se dejó de recibir telemetría del ESP32");
  }
}, 500);

paintSlider(0);
draw();
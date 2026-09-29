/* NeuroCar · interface v2. Polls /estado several times per second and draws.
   It stores nothing: all the state lives on the Pi (or in simulator.js).

   CONTRACT WITH THE PI (what this page reads from /estado):
     telem.*        the ESP CSV (dN..dO, pN..pO, nr0..3, mr0..3, mem, ret,
                    yaw, yaw_ref, cuad, corr, cm_tramo, celdas, estado, ev,
                    rpm_/obj_/pwm_/desp_/agc_ per wheel, hab, imu_ok...)
     corrida.*      cell, orientation, heading, step, reason, figure, mode...
     mapa, traza    discovered walls and last cells
     neuro          (NEW, published by the navigator after deciding) features,
                    h1, h2, q, desglose{N,E,S,O}{tabla,red,patron,momentum,neuro},
                    score[4], probs{N..O}, candidatas, elegida, temperatura,
                    beta, motivo, heading. If it is missing, the Pi side of the
                    network stays off and everything else works.
     decision       (NEW) {direccion, motivo} of the last decision
     celda_cm       (NEW, optional) cell size; if missing, params.celda
     params, estado_esp, ultimo_cmd, hace_cmd, camara_puerto, avisos...

   Direction keys follow the firmware convention: N, E, S, O (O = West).
*/

const $ = (id) => document.getElementById(id);
const DIRS = ["N", "E", "S", "O"];
const REL_NOMBRE = ["front", "right", "back", "left"];
const RUEDAS = ["DI", "DD", "TI", "TD"];
const COLOR = { N: "#22d3ee", E: "#a78bfa", S: "#f472b6", O: "#a3e635",
                DI: "#22d3ee", DD: "#a78bfa", TI: "#f472b6", TD: "#a3e635" };
const CELDA_CM_DEFECTO = 24;
// Display names for the state/mode/event values the Pi publishes (the values
// themselves are protocol keys and stay as they are).
const TXT = { esperando: "waiting", corriendo: "running", terminado: "finished", error: "error",
              idle: "idle", avanzando: "moving forward", girando: "turning", centrando: "centering",
              bloqueado: "stalled", calibrando: "calibrating", manual: "manual",
              exploracion: "exploration", explotacion: "exploitation",
              nada: "none", ack: "ack", nack: "nack", fin_celda: "end of cell", pared: "wall",
              campo_abierto: "open field", bloqueo: "stall", fin_giro: "end of turn",
              err_giro: "turn error", rueda_cortada: "wheel cut off", desalineado: "misaligned" };
const tr = (v) => (v in TXT ? TXT[v] : v);

let historia = [];              // last 20 s of telemetry
let tPrevio = 0, hz = 0;
let ultimoEstado = null;

const fmt = (v, d = 1) => (v == null || isNaN(v)) ? "—" : (+v).toFixed(d);
const clip = (v, a, b) => Math.max(a, Math.min(b, v));

// ------------------------------------------------------------ heat map
// blue = minimum activity, red = maximum; exactly 0 = off (grey).
const PARADAS = [[0.0, [27, 34, 51]], [0.25, [37, 99, 235]], [0.5, [34, 211, 238]],
                 [0.75, [255, 210, 63]], [1.0, [255, 59, 59]]];
function calor(a) {
  a = clip(+a || 0, 0, 1);
  if (a < 0.02) return "rgb(27,34,51)";
  for (let i = 1; i < PARADAS.length; i++) {
    if (a <= PARADAS[i][0]) {
      const [t0, c0] = PARADAS[i - 1], [t1, c1] = PARADAS[i];
      const f = (a - t0) / (t1 - t0);
      const c = c0.map((v, k) => Math.round(v + (c1[k] - v) * f));
      return `rgb(${c[0]},${c[1]},${c[2]})`;
    }
  }
  return "rgb(255,59,59)";
}

// ------------------------------------------------------------ commands
async function orden(accion, extra = {}) {
  try {
    const r = await fetch("/orden", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify(Object.assign({ accion }, extra)),
    });
    let datos = {};
    try { datos = await r.json(); } catch (_) { /* empty */ }
    if (r.ok) { $("aviso-orden").hidden = true; return { ok: true, status: r.status, datos }; }
    mostrarAviso((datos && datos.error) || ("error HTTP " + r.status));
    return { ok: false, status: r.status, datos };
  } catch (err) {
    mostrarAviso("no connection with the Pi (" + err.message + ")");
    return { ok: false, status: 0, datos: {} };
  }
}
function mostrarAviso(texto) { const el = $("aviso-orden"); el.textContent = "⚠ " + texto; el.hidden = false; }

$("btn-start").onclick = () => orden("start");
$("btn-paro").onclick = () => orden("paro");
$("btn-meta").onclick = () => orden("final");
$("btn-limpiar").onclick = async () => {
  if (!confirm("Erase the runs, the Q table, the master route and the map? (the network is kept)")) return;
  const todo = confirm("ALSO reset the neural network and the patterns (everything from scratch, without any bias)? OK = everything from scratch · Cancel = runs only");
  const r = await orden("limpiar", { todo });
  if (r && r.ok) mostrarAviso(todo ? "runs and network erased: from scratch" : "runs erased (the network is kept)");
};
document.addEventListener("keydown", (e) => {
  if (e.target.tagName === "INPUT") return;
  if (e.code === "Space") { e.preventDefault(); orden("paro"); pulsar("stop", false); }
  if (e.key === "f" || e.key === "F") { orden("final"); }
}, true);

// ------------------------------------------------------------ manual control
// Forward/reverse = 5 s push (#t) through the same speed and heading loop as
// navigation. Turns = 90-degree primitive closed by the IMU. Stop cuts the
// push immediately. Only with the robot stopped.
const VX_MANUAL = 140, MS_TIRADA = 5000;
const MANDOS = {
  avance: () => orden("manual_pwm", { vx: VX_MANUAL, vy: 0, w: 0, ms: MS_TIRADA }),
  retro: () => orden("manual_pwm", { vx: -VX_MANUAL, vy: 0, w: 0, ms: MS_TIRADA }),
  izq: () => orden("manual", { letra: "L" }),
  der: () => orden("manual", { letra: "R" }),
  stop: async () => { await orden("manual_pwm", { vx: 0, vy: 0, w: 0 }); return orden("manual", { letra: "0" }); },
};
let mandoActivo = null, mandoTimer = null;
async function pulsar(nombre, avisar = true) {
  const b = document.querySelector(`.mando-btn[data-mando="${nombre}"]`);
  if (!b) return;
  document.querySelectorAll(".mando-btn.activo").forEach((x) => x.classList.remove("activo"));
  if (mandoTimer) { clearTimeout(mandoTimer); mandoTimer = null; }
  b.classList.add("activo"); mandoActivo = nombre;
  const dur = nombre === "avance" || nombre === "retro" ? MS_TIRADA : (nombre === "stop" ? 400 : 1800);
  b.style.setProperty("--dur", dur + "ms");
  mandoTimer = setTimeout(() => { b.classList.remove("activo"); mandoActivo = null; }, dur);
  const r = await MANDOS[nombre]();
  const txt = $("txt-manual");
  if (!r.ok) { b.classList.remove("activo"); txt.textContent = "rejected: " + ((r.datos && r.datos.error) || "no answer"); }
  else if (avisar) txt.textContent = { avance: "forward · 5 s push at " + VX_MANUAL + "/255", retro: "reverse · 5 s push",
    izq: "90° turn to the left, closed by the IMU", der: "90° turn to the right, closed by the IMU", stop: "stopped" }[nombre];
}
document.querySelectorAll(".mando-btn").forEach((b) => {
  const barra = document.createElement("i"); barra.className = "barra-tiempo"; b.appendChild(barra);
  b.onclick = () => pulsar(b.dataset.mando);
});
document.addEventListener("keydown", (e) => {
  if (e.target.tagName === "INPUT" || e.repeat) return;
  const m = { ArrowUp: "avance", ArrowDown: "retro", ArrowLeft: "izq", ArrowRight: "der" }[e.key];
  if (m) { e.preventDefault(); pulsar(m); }
});
// gradient for the SVG strokes of the buttons (a single shared <defs>)
(function () {
  const svg = document.createElementNS("http://www.w3.org/2000/svg", "svg");
  svg.setAttribute("width", "0"); svg.setAttribute("height", "0"); svg.style.position = "absolute";
  svg.innerHTML = `<defs><linearGradient id="degrTrazo" x1="0" y1="0" x2="1" y2="1">
    <stop offset="0" stop-color="#22d3ee"/><stop offset="1" stop-color="#a3ff5c"/></linearGradient></defs>`;
  document.body.prepend(svg);
})();

// ------------------------------------------------------------ neural network
function nodo(id, nombre, sub) {
  const el = document.createElement("div");
  el.className = "neurona"; el.dataset.id = id;
  el.innerHTML = `<i class="nucleo"></i><div class="cuerpo"><div class="l1"><b>${nombre}</b><b class="valor">—</b></div><small class="sub" data-sub="${sub}">${sub}</small></div>`;
  return el;
}
function ponerNodo(el, a, texto, sub) {
  a = clip(+a || 0, 0, 1);
  el.style.setProperty("--a", a.toFixed(3));
  el.style.setProperty("--col", calor(a));
  el.dataset.viva = a > 0.05 ? "1" : "0";
  el.querySelector(".valor").textContent = texto;
  const sb = el.querySelector(".sub");
  sb.innerHTML = sub ? `<em>${sub}</em> · ${sb.dataset.sub}` : sb.dataset.sub;
}
const NODOS = {};
function montarRed() {
  const cap = {
    tof: [["dN", "front", "sensor N"], ["dE", "right", "sensor E"], ["dS", "back", "sensor S"], ["dO", "left", "sensor O"]],
    naka: [["nr0", "z₀ front", "τ 0.35 s"], ["nr1", "z₁ right", "τ 0.35 s"], ["nr2", "z₂ back", "τ 0.35 s"], ["nr3", "z₃ left", "τ 0.35 s"]],
    gauss: [["mr0", "g front", "modulates vₓ"], ["mr1", "g right", "exc 0.4 · inh 0.6"], ["mr2", "g back", "exc 0.4 · inh 0.6"], ["mr3", "g left", "exc 0.4 · inh 0.6"]],
    mem: [["mem", "z₈ memory", "τ 0.5 s"], ["ret", "z₉ reverse", "> 0.55"]],
    votos: [["v_tabla", "Q table", "TD(λ)"], ["v_red", "Q network", "β·MLP"], ["v_patron", "Pattern", "local signature"], ["v_momentum", "Momentum", "keep / turn"], ["v_neuro", "Neuro", "0.4·g from the ESP"]],
    softmax: [["pN", "North", "P(N)"], ["pE", "East", "P(E)"], ["pS", "South", "P(S)"], ["pO", "West", "P(O)"]],
  };
  Object.entries(cap).forEach(([k, lista]) => {
    const cont = $("capa-" + k);
    lista.forEach(([id, n, s]) => { const el = nodo(id, n, s); NODOS[id] = el; cont.appendChild(el); });
  });
  for (const m of ["mlp-h1", "mlp-h2"]) { const c = $(m); for (let i = 0; i < 32; i++) c.appendChild(document.createElement("i")); }
  $("mlp-q").innerHTML = DIRS.map((d) => `<span data-d="${d}">—<small>Q(${d})</small></span>`).join("");
}
montarRed();

// Wires: from each node of one layer to each node of the next. The opacity of
// the wire follows the activity of the SOURCE node, so one can see where the
// signal "flows". The Naka->Gauss connections reproduce the real
// connectivity (own, neighbours at 90 = excitation, opposite = inhibition).
const CABLES = [];
function armarCables() {
  const svg = $("cables"); svg.innerHTML = ""; CABLES.length = 0;
  const pares = [];
  const ids = { tof: ["dN", "dE", "dS", "dO"], naka: ["nr0", "nr1", "nr2", "nr3"], gauss: ["mr0", "mr1", "mr2", "mr3"],
                pi: ["mem", "ret", "v_tabla", "v_red", "v_patron", "v_momentum", "v_neuro"], softmax: ["pN", "pE", "pS", "pO"] };
  ids.tof.forEach((a, i) => pares.push([a, ids.naka[i], "propia"]));
  for (let i = 0; i < 4; i++) {
    pares.push([ids.naka[i], ids.gauss[i], "propia"]);
    pares.push([ids.naka[(i + 3) % 4], ids.gauss[i], "excit"]);
    pares.push([ids.naka[(i + 1) % 4], ids.gauss[i], "excit"]);
    pares.push([ids.naka[(i + 2) % 4], ids.gauss[i], "inhib"]);
  }
  ids.gauss.forEach((g) => { pares.push([g, "v_neuro", "propia"]); pares.push([g, "mlp", "propia"]); });
  ["mem", "ret"].forEach((m) => pares.push([m, "mlp", "propia"]));
  ["v_tabla", "v_red", "v_patron", "v_momentum", "v_neuro"].forEach((v) => ids.softmax.forEach((p) => pares.push([v, p, "propia"])));
  pares.push(["mlp", "v_red", "propia"]);
  pares.forEach(([de, a, tipo]) => {
    const p = document.createElementNS("http://www.w3.org/2000/svg", "path");
    p.dataset.de = de; p.dataset.a = a; p.dataset.tipo = tipo;
    p.setAttribute("fill", "none"); p.setAttribute("stroke-width", tipo === "propia" ? "1.4" : "1");
    svg.appendChild(p); CABLES.push(p);
  });
  colocarCables();
}
function centroDe(id, lado) {
  const el = id === "mlp" ? $("mlp") : NODOS[id];
  const r = el.getBoundingClientRect(), R = $("red").getBoundingClientRect();
  return { x: (lado === "der" ? r.right : r.left) - R.left, y: r.top + r.height / 2 - R.top };
}
function colocarCables() {
  const R = $("red").getBoundingClientRect();
  $("cables").setAttribute("viewBox", `0 0 ${R.width} ${R.height}`);
  CABLES.forEach((p) => {
    const a = centroDe(p.dataset.de, "der"), b = centroDe(p.dataset.a, "izq");
    const dx = Math.max(24, (b.x - a.x) * 0.5);
    p.setAttribute("d", `M${a.x},${a.y} C${a.x + dx},${a.y} ${b.x - dx},${b.y} ${b.x},${b.y}`);
  });
}
window.addEventListener("resize", colocarCables);
const ACT = {};   // current activity per id, for the wires
function pintarCables() {
  CABLES.forEach((p) => {
    const a = clip(ACT[p.dataset.de] || 0, 0, 1);
    const tipo = p.dataset.tipo;
    const col = tipo === "inhib" ? "255,77,94" : (tipo === "excit" ? "163,255,92" : "34,211,238");
    p.setAttribute("stroke", `rgba(${col},${(0.06 + a * 0.6).toFixed(2)})`);
    if (tipo !== "propia") p.setAttribute("stroke-dasharray", tipo === "inhib" ? "3 4" : "6 4");
  });
}

function pintarRed(e) {
  const t = e.telem || {}, hay = Object.keys(t).length > 0;
  // --- ESP (10 Hz)
  DIRS.forEach((d, i) => {
    const mm = hay ? (t["d" + d] ?? 0) : null;
    const a = mm == null ? 0 : clip(1 - mm / 600, 0, 1);
    ACT["d" + d] = a;
    ponerNodo(NODOS["d" + d], a, mm == null ? "—" : fmt(mm, 0), mm == null ? "" : (t["p" + d] > 0 ? "wall" : "free"));
    const nr = hay ? (t["nr" + i] ?? 0) : 0, mr = hay ? (t["mr" + i] ?? 0) : 0;
    ACT["nr" + i] = nr; ACT["mr" + i] = mr;
    ponerNodo(NODOS["nr" + i], nr, hay ? fmt(nr, 2) : "—");
    ponerNodo(NODOS["mr" + i], mr, hay ? fmt(mr, 2) : "—", i === 0 && hay ? "vₓ×" + fmt(clip(mr, 0.35, 1), 2) : "");
  });
  ACT.mem = hay ? t.mem : 0; ACT.ret = hay ? t.ret : 0;
  ponerNodo(NODOS.mem, ACT.mem, hay ? fmt(t.mem, 2) : "—");
  ponerNodo(NODOS.ret, ACT.ret, hay ? fmt(t.ret, 2) : "—", hay && t.ret > 0.55 ? "allowed" : "");

  // --- Pi (per decision)
  const n = e.neuro;
  const h1 = $("mlp-h1").children, h2 = $("mlp-h2").children;
  if (n && n.h1) {
    for (let i = 0; i < 32; i++) { h1[i].style.background = calor(Math.abs(n.h1[i] ?? 0)); h2[i].style.background = calor(Math.abs(n.h2[i] ?? 0)); }
    const qmax = Math.max(1e-6, ...n.q.map(Math.abs));
    $("mlp-q").querySelectorAll("span").forEach((s, i) => { s.style.background = calor(Math.abs(n.q[i]) / qmax * 0.9); s.innerHTML = fmt(n.q[i], 2) + `<small>Q(${DIRS[i]})</small>`; });
    const actMlp = n.h2.reduce((s, v) => s + Math.abs(v), 0) / 32;
    ACT.mlp = actMlp; $("mlp").style.setProperty("--mlp-glow", (actMlp * 30).toFixed(0) + "px");

    // votes: the contribution of each memory to the chosen direction, scaled
    // by the largest absolute contribution of that decision.
    const el = n.elegida, des = n.desglose || {};
    const partes = des[el] || {};
    const escala = Math.max(0.2, ...Object.values(des).flatMap((p) => Object.values(p).map(Math.abs)));
    ["tabla", "red", "patron", "momentum", "neuro"].forEach((k) => {
      const v = partes[k] ?? 0; ACT["v_" + k] = Math.abs(v) / escala;
      ponerNodo(NODOS["v_" + k], Math.abs(v) / escala, (v >= 0 ? "+" : "") + fmt(v, 2), "→ " + el);
    });
    $("txt-temp").textContent = fmt(n.temperatura, 2);
    DIRS.forEach((d) => {
      const p = (n.probs || {})[d] ?? 0, cand = (n.candidatas || []).includes(d);
      ACT["p" + d] = p;
      const nd = NODOS["p" + d];
      ponerNodo(nd, p, fmt(p * 100, 0) + "%", cand ? "" : "forbidden");
      nd.classList.toggle("elegida", d === el);
      nd.classList.toggle("candidata-no", !cand);
    });
  } else {
    for (let i = 0; i < 32; i++) { h1[i].style.background = ""; h2[i].style.background = ""; }
    ["tabla", "red", "patron", "momentum", "neuro"].forEach((k) => { ACT["v_" + k] = 0; ponerNodo(NODOS["v_" + k], 0, "—"); });
    DIRS.forEach((d) => { ACT["p" + d] = 0; ponerNodo(NODOS["p" + d], 0, "—"); NODOS["p" + d].classList.remove("elegida", "candidata-no"); });
    ACT.mlp = 0; $("mlp").style.setProperty("--mlp-glow", "0px"); $("txt-temp").textContent = "—";
  }
  pintarCables();
}

// ------------------------------------------------------------ decision
function pintarDecision(e) {
  const t = e.telem || {}, co = e.corrida || {}, d = e.decision || {};
  const box = $("decision"), big = $("txt-decision"), mot = $("txt-motivo"), hacia = $("txt-hacia");
  box.classList.remove("giro", "parada");
  const estado = e.estado_esp || "";
  const NOMBRE_DIR = { N: "North", E: "East", S: "South", O: "West" };
  let texto = "WAITING", detalle = e.correr ? "observing…" : "waiting for START", dir = null;
  if (d.direccion && co.heading) {
    const rel = (DIRS.indexOf(d.direccion) - DIRS.indexOf(co.heading) + 4) % 4;
    texto = ["FORWARD", "TURN RIGHT", "HALF TURN", "TURN LEFT"][rel];
    dir = d.direccion; detalle = d.motivo || co.motivo || "";
    if (rel) box.classList.add("giro");
  }
  if (estado === "girando") { texto = "TURNING"; box.classList.add("giro"); detalle = "closing by IMU · ref " + fmt(t.yaw_ref, 1) + "°"; }
  else if (estado === "avanzando") { texto = "MOVING FORWARD"; box.classList.remove("giro"); detalle = fmt(t.cm_tramo, 1) + " cm of the stretch · vₓ×" + fmt(clip(t.mr0 ?? 1, 0.35, 1), 2); }
  else if (estado === "bloqueado") { texto = "STALLED"; box.classList.add("parada"); detalle = "odometry stopped with a live setpoint"; }
  else if (estado === "manual") { texto = "MANUAL CONTROL"; dir = null; detalle = "push in progress"; }
  else if (!e.correr && e.estado_pi !== "terminado" && (!estado || estado === "idle")) { texto = "WAITING"; dir = null; box.classList.add("parada"); }
  if (e.estado_pi === "terminado") { texto = "EXIT"; dir = null; detalle = "run closed as a success"; }
  big.textContent = texto; mot.textContent = detalle;
  hacia.hidden = !dir; if (dir) hacia.textContent = "→ " + dir + " · " + NOMBRE_DIR[dir];
  const yaw = Object.keys(t).length ? -(t.yaw ?? 0) : (co.orientacion || 0) * 90;
  $("aguja").style.transform = `translate(-50%,-100%) rotate(${yaw}deg)`;
}

// ------------------------------------------------------------ camera
const ICONO = {
  triangulo_izq: '<path d="M46 10 14 32l32 22z"/>',
  triangulo_der: '<path d="M18 10l32 22-32 22z"/>',
  circulo: '<circle cx="32" cy="32" r="21"/>',
  cuadrado: '<rect x="12" y="12" width="40" height="40" rx="4"/>',
  nada: '<path d="M8 20a24 24 0 0 1 48 0v24a24 24 0 0 1-48 0z" opacity=".35"/><path d="M22 32h20"/>',
};
const NOMBRE_FIG = { triangulo_izq: "triangle ◀ · turn left", triangulo_der: "triangle ▶ · turn right",
                     circulo: "circle · half turn", cuadrado: "square · wait 10 s and re-evaluate" };
let camTimer = 0;
async function pintarCamara(e) {
  const chip = $("chip-cam"), fig = $("cam-figura");
  if (!e.camara_puerto) {
    chip.className = "chip"; chip.querySelector("span").textContent = "off";
    $("cam-icono").innerHTML = ICONO.nada; $("cam-texto").textContent = "camera off"; $("cam-detalle").textContent = "start main.py with --camera";
    fig.classList.remove("ve"); return;
  }
  if (Date.now() - camTimer < 500) return; camTimer = Date.now();
  // the MJPEG preview of the camera server, embedded (before, only on :8081)
  const vista = $("cam-vista");
  if (location.protocol !== "file:" && !vista.src) {
    vista.src = `${location.protocol}//${location.hostname}:${e.camara_puerto}/camara.mjpg`;
    $("cam-enlace").href = `${location.protocol}//${location.hostname}:${e.camara_puerto}/`;
    vista.hidden = false;
  }
  try {
    const url = location.protocol === "file:" ? "/datos" : `${location.protocol}//${location.hostname}:${e.camara_puerto}/datos`;
    const r = await fetch(url); const d = await r.json();
    chip.className = "chip ok"; chip.querySelector("span").textContent = "active · " + fmt(d.dist_frontal, 0) + " mm";
    const f = d.figura || d.avistada;
    if (f) {
      fig.classList.add("ve"); $("cam-icono").innerHTML = ICONO[f] || ICONO.nada;
      $("cam-texto").textContent = NOMBRE_FIG[f] || f;
      $("cam-detalle").textContent = "confidence " + fmt(d.confianza, 2) + " · area " + fmt((d.area_frac || 0) * 100, 1) + "% · " + (d.decidiria ? "OVERRIDES the RL" : "does not pass the gates");
    } else {
      fig.classList.remove("ve"); $("cam-icono").innerHTML = ICONO.nada;
      $("cam-texto").textContent = "no figure in view";
      $("cam-detalle").textContent = d.ultima_figura ? "last obeyed: " + d.ultima_figura : "only looks with a wall in front (< " + fmt(d.dist_decision, 0) + " mm)";
    }
    const puertas = { dist: d.puerta_dist, area: d.puerta_area, conf: d.puerta_conf };
    $("cam-puertas").querySelectorAll("span").forEach((s) => {
      const v = puertas[s.dataset.p]; s.className = v == null ? "" : (v ? "pasa" : "falla");
    });
  } catch (_) {
    chip.className = "chip mal"; chip.querySelector("span").textContent = "no answer";
  }
}

// ------------------------------------------------------------ map
// Reconstruction with odometry: the cell only changes with fin_celda, but
// WITHIN the stretch the robot is drawn at its continuous position (cm_tramo
// along the orientation, rotated by the real yaw). Each ToF reading below
// 60 cm is recorded as a HIT in maze coordinates: over time those points
// draw the walls more precisely than the cell.
const impactos = [];            // [{x,y,a}] in cells
const rastro = [];              // continuous poses
let corridaVista = -1, vista = null;
function poseRobot(e, celdaCm) {
  const t = e.telem || {}, co = e.corrida || {};
  const cel = co.celda || [0, 0], o = co.orientacion || 0;
  const avanzando = (e.estado_esp === "avanzando" || e.estado_esp === "manual");
  const f = avanzando ? clip((t.cm_tramo || 0) / celdaCm, -1, 1.2) : 0;
  const dx = [0, 1, 0, -1][o], dy = [-1, 0, 1, 0][o];
  // on-screen angle (clockwise from North): orientation*90 - yaw deviation
  const desvio = Object.keys(t).length ? ((t.yaw ?? 0) - (t.cuad ?? (t.yaw_ref ?? 0))) : 0;
  const ang = o * 90 - desvio;
  return { x: cel[0] + dx * f, y: cel[1] + dy * f, ang, cel };
}
function pintarMapa(e) {
  const c = $("lienzo"), g = c.getContext("2d");
  const lado = e.lado || 16;
  const celdaCm = e.celda_cm || (e.params && e.params.celda) || CELDA_CM_DEFECTO;
  $("txt-celda-cm").textContent = fmt(celdaCm, 0);
  const co = e.corrida || {}, t = e.telem || {};
  if (co.n !== corridaVista) { corridaVista = co.n; impactos.length = 0; rastro.length = 0; }
  const p = poseRobot(e, celdaCm);
  const hayTelem = Object.keys(t).length > 0;
  // Sep 16: no ToF hits or odometric trace (they cluttered the map). Only the
  // walls recorded by the Pi, the anti-loop trace and the robot are drawn.

  // --- framing: the box of what was visited, with a margin, minimum 7 cells;
  // it is interpolated so the map does not jump when the robot discovers a cell.
  let xs = [p.x], ys = [p.y];
  Object.keys(e.mapa || {}).forEach((k) => { const [x, y] = k.split(",").map(Number); xs.push(x); ys.push(y); });
  rastro.forEach((r) => { xs.push(r.x); ys.push(r.y); });
  let x0 = Math.min(...xs) - 1.5, x1 = Math.max(...xs) + 1.5, y0 = Math.min(...ys) - 1.5, y1 = Math.max(...ys) + 1.5;
  const span = Math.max(7, x1 - x0, y1 - y0);
  const cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
  const objetivo = { cx, cy, span };
  if (!vista) vista = objetivo;
  else { vista.cx += (objetivo.cx - vista.cx) * 0.08; vista.cy += (objetivo.cy - vista.cy) * 0.08; vista.span += (objetivo.span - vista.span) * 0.08; }
  const W = c.width, paso = W / vista.span;
  const X = (x) => (x - vista.cx) * paso + W / 2 + paso / 2 - paso / 2;
  const Y = (y) => (y - vista.cy) * paso + W / 2;
  // cells: center at (x,y), corner at (x-.5, y-.5)
  g.clearRect(0, 0, W, W);

  // grid of the whole board
  g.strokeStyle = "rgba(148,173,214,.07)"; g.lineWidth = 1;
  for (let i = 0; i <= lado; i++) {
    g.beginPath(); g.moveTo(X(i - 0.5), Y(-0.5)); g.lineTo(X(i - 0.5), Y(lado - 0.5)); g.stroke();
    g.beginPath(); g.moveTo(X(-0.5), Y(i - 0.5)); g.lineTo(X(lado - 0.5), Y(i - 0.5)); g.stroke();
  }
  // board border
  g.strokeStyle = "rgba(148,173,214,.22)"; g.lineWidth = 1.5;
  g.strokeRect(X(-0.5), Y(-0.5), lado * paso, lado * paso);
  // coordinates
  g.fillStyle = "rgba(92,107,138,.9)"; g.font = `${Math.max(9, paso * 0.22)}px JetBrains Mono, monospace`; g.textAlign = "center"; g.textBaseline = "middle";
  for (let i = 0; i < lado; i++) { if (X(i) > 0 && X(i) < W) { g.fillText(i, X(i), Y(-0.5) - 8); g.fillText(i, X(i), Y(lado - 0.5) + 8); } }
  g.textAlign = "right";
  for (let i = 0; i < lado; i++) { if (Y(i) > 0 && Y(i) < W) { g.fillText(i, X(-0.5) - 6, Y(i)); } }

  // visited cells
  Object.keys(e.mapa || {}).forEach((k) => {
    const [x, y] = k.split(",").map(Number);
    g.fillStyle = "rgba(34,211,238,.06)";
    g.fillRect(X(x - 0.5) + 1, Y(y - 0.5) + 1, paso - 2, paso - 2);
  });
  // start
  const ini = [Math.floor(lado / 2), Math.floor(lado / 2)];
  g.strokeStyle = "rgba(124,92,255,.7)"; g.lineWidth = 1.5; g.setLineDash([4, 4]);
  g.strokeRect(X(ini[0] - 0.5) + 4, Y(ini[1] - 0.5) + 4, paso - 8, paso - 8); g.setLineDash([]);

  // discovered walls
  g.strokeStyle = "rgba(232,238,251,.75)"; g.lineWidth = Math.max(1.5, paso * 0.05); g.lineCap = "round";
  g.shadowBlur = 0;
  Object.entries(e.mapa || {}).forEach(([k, pa]) => {
    const [x, y] = k.split(",").map(Number);
    const l = X(x - 0.5), r = X(x + 0.5), tt = Y(y - 0.5), b = Y(y + 0.5);
    const seg = { N: [l, tt, r, tt], S: [l, b, r, b], O: [l, tt, l, b], E: [r, tt, r, b] };
    DIRS.forEach((d) => { if (!pa[d]) return; const s = seg[d]; g.beginPath(); g.moveTo(s[0], s[1]); g.lineTo(s[2], s[3]); g.stroke(); });
  });
  g.shadowBlur = 0;

  // forbidden cells (anti-loop window)
  g.fillStyle = "rgba(124,92,255,.35)";
  (e.traza || []).forEach(([x, y]) => { g.beginPath(); g.arc(X(x), Y(y), paso * 0.08, 0, Math.PI * 2); g.fill(); });

  // robot
  const rx = X(p.x), ry = Y(p.y), a = p.ang * Math.PI / 180;
  // ToF beams (chassis frame)
  DIRS.forEach((d, i) => {
    const mm = t["d" + d]; if (mm == null) return;
    const aa = a + i * Math.PI / 2, r = Math.min(mm, 600) / 10 / celdaCm * paso;
    if (!(t["p" + d] > 0)) return;            // only those that see a wall
    g.strokeStyle = "rgba(255,77,94,.25)"; g.lineWidth = 1;
    g.beginPath(); g.moveTo(rx, ry); g.lineTo(rx + Math.sin(aa) * r, ry - Math.cos(aa) * r); g.stroke();
  });
  g.save(); g.translate(rx, ry); g.rotate(a);
  const s = paso * 0.36;
  g.fillStyle = "rgba(11,17,28,.9)"; g.strokeStyle = "#22d3ee"; g.lineWidth = 2;
  g.beginPath(); g.roundRect(-s * 0.7, -s, s * 1.4, s * 2, s * 0.25); g.fill(); g.stroke();
  const gd = g.createLinearGradient(0, -s, 0, s * 0.2); gd.addColorStop(0, "#a3ff5c"); gd.addColorStop(1, "#22d3ee");
  g.fillStyle = gd; g.beginPath(); g.moveTo(0, -s * 1.15); g.lineTo(s * 0.45, -s * 0.25); g.lineTo(-s * 0.45, -s * 0.25); g.closePath(); g.fill();
  g.restore();

  $("txt-celda").textContent = (co.celda || [0, 0]).join(",");
  $("txt-orient").textContent = DIRS[co.orientacion || 0];
  $("txt-paso").textContent = co.paso || 0;
  $("txt-cm").textContent = fmt(t.cm_tramo, 1);
}

// ------------------------------------------------------------ plots
const GRAFICAS = {
  "g-tof": { series: DIRS.map((d) => ({ n: "ToF " + d, k: "d" + d, c: COLOR[d] })), min: 0, max: 1200, umbral: (e) => DIRS.map((d) => ({ v: (e.params || {})["umbral" + d] ?? 200, c: COLOR[d] })), ley: "ley-tof", u: " mm" },
  "g-gauss": { series: DIRS.map((d, i) => ({ n: "g " + d, k: "mr" + i, c: COLOR[d] })), min: 0, max: 1, ley: "ley-gauss", u: "", dec: 2 },
  "g-pwm-izq": { series: [{ n: "DI front", k: "pwm_DI", c: COLOR.DI }, { n: "TI rear", k: "pwm_TI", c: COLOR.TI }], min: 0, max: 255, ley: "ley-pwm-izq", u: "" },
  "g-pwm-der": { series: [{ n: "DD front", k: "pwm_DD", c: COLOR.DD }, { n: "TD rear", k: "pwm_TD", c: COLOR.TD }], min: 0, max: 255, ley: "ley-pwm-der", u: "" },
};
let cursorG = null;   // {id, i} sample under the cursor
function montarGraficas() {
  Object.entries(GRAFICAS).forEach(([id, G]) => {
    $(G.ley).innerHTML = G.series.map((s) => `<span><i style="background:${s.c}"></i>${s.n}</span>`).join("") +
      (G.umbral ? '<span><i class="punteada"></i>threshold</span>' : "");
    const c = $(id);
    c.addEventListener("mousemove", (ev) => {
      const r = c.getBoundingClientRect();
      cursorG = { id, f: (ev.clientX - r.left) / r.width, x: ev.clientX, y: ev.clientY };
    });
    c.addEventListener("mouseleave", () => { cursorG = null; ocultarTooltip(); });
  });
}
montarGraficas();
let tooltip = null;
function ocultarTooltip() { if (tooltip) { tooltip.remove(); tooltip = null; } }
function pintarGrafica(id, e) {
  const G = GRAFICAS[id], c = $(id), g = c.getContext("2d");
  c.width = c.clientWidth * devicePixelRatio; c.height = 170 * devicePixelRatio;
  g.scale(devicePixelRatio, devicePixelRatio);
  const W = c.clientWidth, H = 170, ml = 34, mb = 16, mt = 6;
  const Yv = (v) => mt + (1 - (clip(v, G.min, G.max) - G.min) / (G.max - G.min)) * (H - mt - mb);
  g.clearRect(0, 0, W, H);
  // recessive grid and axis
  g.strokeStyle = "rgba(148,173,214,.08)"; g.lineWidth = 1; g.fillStyle = "rgba(92,107,138,.9)"; g.font = "10px JetBrains Mono, monospace"; g.textAlign = "right"; g.textBaseline = "middle";
  [0, 0.5, 1].forEach((f) => { const v = G.min + f * (G.max - G.min), y = Yv(v); g.beginPath(); g.moveTo(ml, y); g.lineTo(W, y); g.stroke(); g.fillText(G.dec ? v.toFixed(1) : v.toFixed(0), ml - 6, y); });
  g.textAlign = "center"; g.textBaseline = "top"; g.fillText("−20 s", ml + 18, H - mb + 4); g.fillText("now", W - 18, H - mb + 4);
  if (G.umbral) {
    g.setLineDash([3, 4]);
    G.umbral(e).forEach((u) => { g.strokeStyle = u.c + "88"; g.beginPath(); g.moveTo(ml, Yv(u.v)); g.lineTo(W, Yv(u.v)); g.stroke(); });
    g.setLineDash([]);
  }
  const n = historia.length; if (n < 2) return;
  const Xi = (i) => ml + (i / 99) * (W - ml);
  const desde = 100 - n;
  G.series.forEach((s) => {
    g.strokeStyle = s.c; g.lineWidth = 2; g.lineJoin = "round"; g.shadowColor = s.c; g.shadowBlur = 6;
    g.beginPath();
    historia.forEach((h, i) => { const x = Xi(desde + i), y = Yv(h[s.k] ?? 0); i ? g.lineTo(x, y) : g.moveTo(x, y); });
    g.stroke(); g.shadowBlur = 0;
  });
  // cursor
  if (cursorG && cursorG.id === id) {
    const i = clip(Math.round(cursorG.f * 99) - desde, 0, n - 1);
    const x = Xi(desde + i);
    g.strokeStyle = "rgba(232,238,251,.35)"; g.lineWidth = 1; g.beginPath(); g.moveTo(x, mt); g.lineTo(x, H - mb); g.stroke();
    G.series.forEach((s) => { const y = Yv(historia[i][s.k] ?? 0); g.fillStyle = s.c; g.beginPath(); g.arc(x, y, 4, 0, Math.PI * 2); g.fill(); g.strokeStyle = "#0b111c"; g.lineWidth = 2; g.stroke(); });
    if (!tooltip) { tooltip = document.createElement("div"); tooltip.className = "tooltip-g"; document.body.appendChild(tooltip); }
    tooltip.innerHTML = G.series.map((s) => `<div><span><i style="background:${s.c}"></i>${s.n}</span><b>${fmt(historia[i][s.k], G.dec || 0)}${G.u}</b></div>`).join("");
    tooltip.style.left = (cursorG.x + 14) + "px"; tooltip.style.top = (cursorG.y - 10) + "px";
  }
}

// ------------------------------------------------------------ telemetry
function grupo(titulo, filas) {
  return `<div class="tele-grupo"><h4>${titulo}</h4>` + filas.map(([k, v, cls]) =>
    `<div class="fila"><span>${k}</span><b class="${cls || ""}">${v}</b></div>`).join("") + "</div>";
}
function pintarTelemetria(e) {
  const t = e.telem || {}, p = e.params || {}, co = e.corrida || {};
  const rue = e.ruedas || [];
  const NOMBRE_CMD = { "0": "0 stop", "5": "5 forward", "9": "9 center", "R": "R turn right", "L": "L turn left", "T": "T half turn", "H": "H HALT", "A": "A up to wall", "Y": "Y align", "Z": "Z set heading", "E": "E motors" };
  const cmd = e.ultimo_cmd || "";
  $("telemetria").innerHTML =
    grupo("Heading · IMU", [
      ["yaw", fmt(t.yaw, 2) + "<em>°</em>"], ["reference", fmt(t.yaw_ref, 2) + "<em>°</em>"],
      ["grid", fmt(t.cuad, 1) + "<em>°</em>"], ["correction", fmt(t.corr, 1) + "<em>rpm</em>"],
      ["last turn error", fmt(t.err_giro, 2) + "<em>°</em>", Math.abs(t.err_giro || 0) > 4.4 ? "aviso" : ""],
      ["centering error", fmt(t.ecen, 1) + "<em>mm</em>"],
    ]) +
    grupo("ToF · wall thresholds", DIRS.map((d) => [
      `${d} · ${REL_NOMBRE[DIRS.indexOf(d)]}`, `${fmt(t["d" + d], 0)}<em>/ ${fmt(p["umbral" + d] ?? 200, 0)} mm</em>`, t["p" + d] > 0 ? "aviso" : ""])) +
    grupo("Encoders · wheels", rue.length ? rue.map((r) => [
      `${r.nombre} rpm`, `${fmt(r.rpm, 0)}<em>/ ${fmt(r.objetivo, 0)} · pwm ${fmt(r.pwm, 0)}</em>`, r.cortada ? "mal" : (!r.viva ? "aviso" : "")])
      .concat([["cm of the stretch", fmt(t.cm_tramo, 1) + "<em>cm</em>"], ["cells counted", fmt(t.celdas, 0)], ["odometry factor", fmt(p.odom, 2)]])
      : [["no data", "—"]]) +
    grupo("Status", [
      ["ESP", tr(e.estado_esp) || "—"], ["last command", cmd ? (NOMBRE_CMD[cmd] || cmd) + `<em>${fmt(e.hace_cmd, 1)} s ago</em>` : "—"],
      ["event", tr(co.evento) || "—"], ["runs", `${co.exitos || 0}<em>/ ${co.corridas || 0} successes</em>`],
      ["mode", tr(co.modo) || "—"], ["softmax temperature", fmt(co.temperatura, 2)],
      ["ESP cycle", fmt(t.ciclo_ms, 0) + "<em>ms</em>", (t.ciclo_ms || 0) > 60 ? "aviso" : ""], ["telemetry", fmt(hz, 1) + "<em>Hz</em>"],
    ]) +
    grupo("ESP messages", (e.avisos || []).slice(-5).reverse().map((a) => ["", `<span style="font-weight:400;font-size:11px;color:var(--tinta-2)">${a}</span>`]).concat((e.avisos || []).length ? [] : [["", "—"]]));
  $("txt-fw").textContent = e.fw ? "fw " + e.fw : "";
}

// ------------------------------------------------------------ header
function pintarCabecera(e) {
  const t = e.telem || {}, hay = Object.keys(t).length > 0;
  const chip = (id, ok, txt, cls) => { const c = $(id); c.className = "chip " + (cls || (ok ? "ok" : "mal")); c.querySelector("span").textContent = txt; };
  chip("chip-enlace", hay, hay ? "link alive" : "no data");
  chip("chip-imu", !!t.imu_ok, t.imu_ok ? "IMU ok" : "no IMU", hay ? "" : "");
  chip("chip-motores", !!t.hab, t.hab ? "motors ON" : "motors off", t.hab ? "ok" : "");
  $("chip-hz").querySelector("b").textContent = hz.toFixed(0);
  $("txt-estado-pi").textContent = tr(e.estado_pi) || "—";
  $("chip-modo").classList.toggle("corriendo", !!e.correr);
  $("btn-start").classList.toggle("activo", !!e.correr);
  $("mensaje").textContent = e.mensaje || "";
}

// ------------------------------------------------------------ loop
let cablesListos = false;
function pintar(e) {
  ultimoEstado = e;
  const t = e.telem || {};
  if (Object.keys(t).length) { historia.push(t); if (historia.length > 100) historia.shift(); }
  if (!cablesListos) { armarCables(); cablesListos = true; }
  pintarCabecera(e);
  pintarRed(e);
  pintarDecision(e);
  pintarCamara(e);
  pintarMapa(e);
  Object.keys(GRAFICAS).forEach((id) => pintarGrafica(id, e));
  pintarTelemetria(e);
}
async function tic() {
  try {
    const r = await fetch("/estado");
    const e = await r.json();
    const t = e.telem || {};
    if (t.t_ms && tPrevio) { const dt = (t.t_ms - tPrevio) / 1000; if (dt > 0) hz = 0.7 * hz + 0.3 * (1 / dt); }
    tPrevio = t.t_ms || tPrevio;
    pintar(e);
  } catch (err) {
    const c = $("chip-enlace"); c.className = "chip mal"; c.querySelector("span").textContent = "no server";
  }
}
// web fonts change the width of the nodes: the wires are repositioned on load
if (document.fonts && document.fonts.ready) document.fonts.ready.then(() => { if (cablesListos) colocarCables(); });
setInterval(tic, 300);   // Sep 16: 3.3 Hz is enough and offloads the Pi
tic();

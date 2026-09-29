/* Simulator to preview the interface WITHOUT the robot.

   It only activates if the page is opened from disk (file://) or with
   ?dummy=1. Then it replaces window.fetch for /estado, /orden and /datos
   (camera) and answers with the SAME format as the Pi, so that app.js is
   identical in the preview and on the robot.

   What it simulates, with the real equations of the firmware and rl.py:
     - random 16x16 maze (24 cm cells) with an exit through one border
     - neurocontroller: Naka-Rushton, gaussians with lateral excitation and
       inhibition of the opposite side, memory and reverse permission (BLOCK 09)
     - MLP 8-32-32-4 with tanh and random weights (only to show activity)
     - score = table + beta*network + pattern + momentum + 0.4*g, softmax with T
     - turns closed "by the IMU" and forward moves with continuous odometry (cm_tramo)
     - PWM/rpm per wheel with ramp and noise
     - camera: every now and then it sees a figure when reaching a wall
*/
(function () {
  const activar = location.protocol === "file:" || /[?&]dummy=1/.test(location.search);
  if (!activar) return;
  // 🔴 VISIBLE WARNING: with the simulator active NOTHING reaches the robot.
  // (Sep 16: the page opened from disk showed data and the robot did not move.)
  document.addEventListener("DOMContentLoaded", () => {
    const b = document.createElement("div");
    b.textContent = "SIMULATOR ACTIVE — this page does NOT talk to the robot. Open http://<pi-ip>:8080";
    b.style.cssText = "position:fixed;top:0;left:0;right:0;z-index:9999;background:#ff3b3b;color:#fff;" +
      "font:700 15px/1.4 system-ui,sans-serif;text-align:center;padding:8px 12px;letter-spacing:.02em";
    document.body.prepend(b);
    document.title = "[SIMULATOR] " + document.title;
  });

  const LADO = 16, INICIO = [8, 8], CELDA_CM = 24, PASO_CM = 24;
  const DIRS = ["N", "E", "S", "O"];
  const DX = { N: 0, E: 1, S: 0, O: -1 }, DY = { N: -1, E: 0, S: 1, O: 0 };
  const OPUESTA = { N: "S", S: "N", E: "O", O: "E" };
  const UMBRAL = 200;

  // ------------------------------------------------------------ maze
  // perfect DFS + a few knocked-down walls (so there are loops and
  // crossings) + an exit through a random border.
  function generarLaberinto() {
    const paredes = {};
    for (let y = 0; y < LADO; y++) for (let x = 0; x < LADO; x++)
      paredes[x + "," + y] = { N: true, E: true, S: true, O: true };
    const vis = new Set(); const pila = [[INICIO[0], INICIO[1]]];
    vis.add(INICIO.join(","));
    while (pila.length) {
      const [x, y] = pila[pila.length - 1];
      const ops = DIRS.filter(d => {
        const nx = x + DX[d], ny = y + DY[d];
        return nx >= 0 && ny >= 0 && nx < LADO && ny < LADO && !vis.has(nx + "," + ny);
      });
      if (!ops.length) { pila.pop(); continue; }
      const d = ops[Math.floor(Math.random() * ops.length)];
      const nx = x + DX[d], ny = y + DY[d];
      paredes[x + "," + y][d] = false; paredes[nx + "," + ny][OPUESTA[d]] = false;
      vis.add(nx + "," + ny); pila.push([nx, ny]);
    }
    for (let k = 0; k < 40; k++) {
      const x = 1 + Math.floor(Math.random() * (LADO - 2)), y = 1 + Math.floor(Math.random() * (LADO - 2));
      const d = DIRS[Math.floor(Math.random() * 4)];
      paredes[x + "," + y][d] = false; paredes[(x + DX[d]) + "," + (y + DY[d])][OPUESTA[d]] = false;
    }
    // exit: a random border, far from the center
    const lado = DIRS[Math.floor(Math.random() * 4)];
    const i = Math.floor(Math.random() * LADO);
    let salida;
    if (lado === "N") salida = [i, 0]; else if (lado === "S") salida = [i, LADO - 1];
    else if (lado === "O") salida = [0, i]; else salida = [LADO - 1, i];
    paredes[salida.join(",")][lado] = false;
    return { paredes, salida, ladoSalida: lado };
  }

  // ------------------------------------------------------------ neurocontroller (BLOCK 09)
  const DT = 0.05, TAU_NR = 0.35, TAU_GA = 0.10, TAU_MEM = 0.5, TAU_RET = 0.8;
  const W_EXCIT = 0.4, W_INHIB = 0.6, SIGMA = 35, UMBRAL_RET = 0.55;
  const naka = x => { x = Math.max(0, x); return (x * x) / (x * x + 0.25 + 1e-12); };
  const gauss = (a, b) => { let d = a - b; while (d > 180) d -= 360; while (d < -180) d += 360; return Math.exp(-0.5 * (d / SIGMA) ** 2); };
  const ang = i => [90, 0, 270, 180][i];
  const clip = (v, a, b) => Math.max(a, Math.min(b, v));
  const z = new Array(10).fill(0);
  function pasoNeuro(dists, haySalida) {
    for (let i = 0; i < 4; i++) z[i] = clip(z[i] + (DT / TAU_NR) * (-z[i] + naka(clip(dists[i] / 1200, 0, 1))), 0, 1);
    const zn = z.slice(0, 4);
    for (let i = 0; i < 4; i++) {
      const il = (i + 3) % 4, ir = (i + 1) % 4, io = (i + 2) % 4;
      const u = clip(gauss(ang(i), ang(i)) * zn[i]
        + W_EXCIT * gauss(ang(i), ang(il)) * zn[il]
        + W_EXCIT * gauss(ang(i), ang(ir)) * zn[ir]
        - W_INHIB * gauss(ang(i), ang(io)) * zn[io], 0, 1);
      z[4 + i] = clip(z[4 + i] + (DT / TAU_GA) * (-z[4 + i] + u), 0, 1);
    }
    const dead = haySalida ? 0 : 1;
    z[8] = clip(z[8] + (DT / TAU_MEM) * (-z[8] + dead), 0, 1);
    z[9] = clip(z[9] + (DT / TAU_RET) * (-z[9] + dead), 0, 1);
  }

  // ------------------------------------------------------------ MLP 8-32-32-4
  const rnd = n => (Math.random() * 2 - 1) / Math.sqrt(n);
  const W1 = Array.from({ length: 32 }, () => Array.from({ length: 8 }, () => rnd(8) * 2.5));
  const W2 = Array.from({ length: 32 }, () => Array.from({ length: 32 }, () => rnd(32) * 2.5));
  const W3 = Array.from({ length: 4 }, () => Array.from({ length: 32 }, () => rnd(32) * 2));
  function mlp(x) {
    const h1 = W1.map(f => Math.tanh(f.reduce((s, w, j) => s + w * x[j], 0)));
    const h2 = W2.map(f => Math.tanh(f.reduce((s, w, j) => s + w * h1[j], 0)));
    const q = W3.map(f => f.reduce((s, w, j) => s + w * h2[j], 0));
    return { h1, h2, q };
  }

  // ------------------------------------------------------------ world state
  let lab = generarLaberinto();
  const S = {
    celda: [...INICIO], orient: 0,            // 0=N 1=E 2=S 3=O(W) (absolute)
    yaw: 0, yawRef: 0, cuad: 0,               // degrees; yaw>0 = left
    loco: 0,                                  // 0 idle 1 forward 2 turning 4 stalled 6 manual
    cm: 0, celdas: 0, ev: 0, cmd: 0,
    hab: false, correr: false, estadoPi: "esperando",
    mensaje: "Simulator: press START to watch the robot explore",
    paso: 0, corridas: 0, exitos: 0, corridaN: 0,
    mapa: {}, traza: [], vistas: new Set(),
    tablaQ: {}, neuro: null, decision: {},
    ruedas: { DI: 0, DD: 0, TI: 0, TD: 0 }, obj: { DI: 0, DD: 0, TI: 0, TD: 0 },
    pwm: { DI: 0, DD: 0, TI: 0, TD: 0 },
    cam: { activa: true, figura: null, confianza: 0, area: 0, avistada: null, ultima: null },
    avisos: ["READY fw=sim-2026-09-15", "ST: idle hab=0 imu=1"],
    tarea: null, tMs: 0, ultimoCmd: "", tCmd: Date.now(),
  };

  // real distance of a ToF (chassis frame) to the wall, with raycasting by
  // cells and the continuous pose within the cell.
  function poseContinua() {
    const d = DIRS[S.orient];
    const f = (S.loco === 1 ? S.cm / CELDA_CM : 0);
    return { x: S.celda[0] + DX[d] * f, y: S.celda[1] + DY[d] * f };
  }
  function distancia(dirChasis) {
    const dAbs = DIRS[(S.orient + DIRS.indexOf(dirChasis)) % 4];
    const p = poseContinua();
    let cx = Math.round(p.x), cy = Math.round(p.y);
    // distance from the pose to the edge of the current cell in that direction
    let off = 0;
    if (dAbs === "N") off = (p.y - (cy - 0.5)); else if (dAbs === "S") off = ((cy + 0.5) - p.y);
    else if (dAbs === "E") off = ((cx + 0.5) - p.x); else off = (p.x - (cx - 0.5));
    let mm = off * CELDA_CM * 10;
    for (let k = 0; k < 6; k++) {
      const c = lab.paredes[cx + "," + cy];
      if (!c || c[dAbs]) return Math.min(1200, mm + (Math.random() - 0.5) * 6);
      cx += DX[dAbs]; cy += DY[dAbs]; mm += CELDA_CM * 10;
      if (cx < 0 || cy < 0 || cx >= LADO || cy >= LADO) return 1200;
    }
    return 1200;
  }

  function paredesAbs() {
    const p = {};
    for (let i = 0; i < 4; i++) {
      const dAbs = DIRS[(S.orient + i) % 4];
      p[dAbs] = distancia(DIRS[i]) < UMBRAL ? 1 : 0;
    }
    return p;
  }

  // ------------------------------------------------------------ decision (rl.py)
  function decidir() {
    const k = S.celda.join(",");
    const paredes = paredesAbs();
    S.mapa[k] = paredes; S.vistas.add(k);
    const libres = DIRS.filter(d => !paredes[d]);
    const recientes = S.traza.slice(-6).map(c => c.join(","));
    let cand = libres.filter(d => !recientes.includes((S.celda[0] + DX[d]) + "," + (S.celda[1] + DY[d])));
    if (!cand.length) cand = libres;
    const heading = DIRS[S.orient];
    const g = {}; for (let i = 0; i < 4; i++) g[DIRS[(S.orient + i) % 4]] = z[4 + i];
    const dist = {}; for (let i = 0; i < 4; i++) dist[DIRS[(S.orient + i) % 4]] = distancia(DIRS[i]);
    const x = [dist.N / 1200, dist.E / 1200, dist.S / 1200, dist.O / 1200,
      (15 - S.celda[0]) / LADO, (15 - S.celda[1]) / LADO, z[8], z[9]].map(v => clip(v, -1, 1));
    const red = mlp(x);
    const beta = Math.min(1, 0.1 + 0.05 * S.corridas);
    const T = Math.max(0.1, 0.6 - 0.06 * S.corridas);
    const tab = S.tablaQ[k] || (S.tablaQ[k] = [0, 0, 0, 0]);
    const desglose = {}, score = [];
    DIRS.forEach((d, i) => {
      const rel = (i - S.orient + 4) % 4;
      const mom = rel === 0 ? 0.25 : rel === 2 ? -0.55 : 0.19;
      const partes = { tabla: tab[i], red: beta * red.q[i], patron: 0.6 * (Math.random() * 0.3 - 0.1), momentum: mom, neuro: 0.4 * g[d] };
      desglose[d] = partes; score[i] = Object.values(partes).reduce((a, b) => a + b, 0);
    });
    const idx = cand.map(d => DIRS.indexOf(d));
    const mx = Math.max(...idx.map(i => score[i]));
    const ex = idx.map(i => Math.exp((score[i] - mx) / T)); const sum = ex.reduce((a, b) => a + b, 0);
    const probs = {}; DIRS.forEach(d => probs[d] = 0); idx.forEach((i, j) => probs[DIRS[i]] = ex[j] / sum);
    // draw
    let r = Math.random(), elegida = cand[cand.length - 1];
    for (let j = 0; j < cand.length; j++) { r -= ex[j] / sum; if (r <= 0) { elegida = cand[j]; break; } }
    let motivo = "softmax(T=" + T.toFixed(2) + ")";

    // camera: if there is a wall in front and we are close, sometimes there is a figure
    S.cam.figura = null; S.cam.avistada = null;
    if (paredes[heading] && Math.random() < 0.28) {
      const figs = ["triangulo_izq", "triangulo_der", "circulo", "cuadrado"];
      const f = figs[Math.floor(Math.random() * 4)];
      S.cam.figura = f; S.cam.avistada = f; S.cam.confianza = 0.6 + Math.random() * 0.4; S.cam.area = 0.08 + Math.random() * 0.2;
      const rel = { triangulo_izq: 3, triangulo_der: 1, circulo: 2 }[f];
      if (rel != null) { const fd = DIRS[(S.orient + rel) % 4]; if (!paredes[fd]) { elegida = fd; motivo = "figura:" + f; S.cam.ultima = f; } }
      else motivo = "cuadrado+" + motivo;
    }
    S.neuro = {
      rasgos: x, h1: red.h1, h2: red.h2, q: red.q, desglose, score, probs,
      candidatas: cand, elegida, temperatura: T, beta, motivo, heading,
      tabla: tab.slice(),
    };
    return elegida;
  }

  // ------------------------------------------------------------ primitives (timed)
  const espera = ms => new Promise(r => setTimeout(r, ms));
  async function girarA(dirAbs) {
    const delta = ((DIRS.indexOf(dirAbs) - S.orient) + 4) % 4;
    if (delta === 0) return true;
    const pasos = delta === 3 ? -1 : delta;           // -1 left, 1 right, 2 half turn
    S.loco = 2; S.cmd = pasos === -1 ? 76 : pasos === 1 ? 82 : 84; S.ultimoCmd = pasos === -1 ? "L" : pasos === 1 ? "R" : "T"; S.tCmd = Date.now();
    const grados = -90 * pasos;                        // yaw>0 = left
    const y0 = S.yaw, dur = 900 + Math.abs(pasos) * 500, t0 = performance.now();
    S.yawRef = y0 + grados;
    while (performance.now() - t0 < dur) {
      const f = Math.min(1, (performance.now() - t0) / dur);
      const e = 1 - Math.pow(1 - f, 3);
      S.yaw = y0 + grados * e + (Math.random() - 0.5) * 0.4;
      const w = 90 * (1 - f) + 25;
      const s = Math.sign(grados);
      S.obj = { DI: -s * w, DD: s * w, TI: -s * w, TD: s * w };
      await espera(50);
    }
    S.yaw = S.yawRef + (Math.random() - 0.5) * 1.2;
    S.orient = DIRS.indexOf(dirAbs); S.cuad = -90 * S.orient;
    S.obj = { DI: 0, DD: 0, TI: 0, TD: 0 };
    S.loco = 0; S.ev = 7; await espera(120); S.ev = 0;
    S.avisos.push("OK: turn " + S.ultimoCmd + " err=" + (S.yaw - S.yawRef).toFixed(2));
    return true;
  }
  async function avanzar() {
    S.cmd = 53; S.ultimoCmd = "5"; S.tCmd = Date.now();
    const d = DIRS[S.orient];
    if (lab.paredes[S.celda.join(",")][d]) { S.ev = 2; await espera(120); S.ev = 0; return 2; }
    S.loco = 1; S.cm = 0;
    const t0 = performance.now();
    let v = 0;
    while (S.cm < PASO_CM) {
      const g = z[4];                                  // front gaussian
      const vObj = 77 * clip(g, 0.35, 1);
      v += (vObj - v) * 0.18;
      S.obj = { DI: v, DD: v, TI: v, TD: v };
      S.cm += v * 0.1 / 60 * Math.PI * 0.0655 * 100 * 1.0; // rpm -> cm per 100 ms (65.5 mm wheel)
      await espera(100);
      if (!S.correr && !S.tarea) { S.loco = 0; S.obj = { DI: 0, DD: 0, TI: 0, TD: 0 }; return 6; }
      if (performance.now() - t0 > 9000) break;
    }
    S.celda = [S.celda[0] + DX[d], S.celda[1] + DY[d]];
    S.celdas++; S.traza.push([...S.celda]); if (S.traza.length > 400) S.traza.shift();
    S.cm = 0; S.loco = 0; S.obj = { DI: 0, DD: 0, TI: 0, TD: 0 };
    S.ev = 3; await espera(120); S.ev = 0;
    return 3;
  }
  function fueraDelLaberinto() {
    const [x, y] = S.celda;
    return x < 0 || y < 0 || x >= LADO || y >= LADO;
  }

  async function bucleCorrida() {
    S.corridaN++; S.paso = 0; S.traza = [[...S.celda]];
    S.estadoPi = "corriendo"; S.mensaje = "Run " + S.corridaN + " in progress (simulated)";
    S.hab = true;
    while (S.correr && S.paso < 400) {
      await espera(500);
      const dir = decidir();
      S.decision = { direccion: dir, motivo: S.neuro.motivo };
      if (S.neuro.motivo.startsWith("cuadrado")) { S.mensaje = "Square: still for 10 s and look again (shortened to 2 s in the simulator)"; await espera(2000); }
      await girarA(dir);
      if (!S.correr) break;
      const ev = await avanzar();
      S.paso++;
      S.mensaje = "Step " + S.paso + " · " + dir + " · event " + { 3: "end of cell", 2: "nack", 6: "stall" }[ev];
      if (fueraDelLaberinto() || (S.celda.join(",") === lab.salida.join(",") && Math.random() < 0.6)) {
        // exit: the four ToF far away
        S.exitos++; S.corridas++; S.correr = false; S.estadoPi = "terminado";
        S.mensaje = "EXIT detected: ToF far away. Run " + S.corridaN + " closed as a success. START begins again.";
        S.celda = [...INICIO]; S.orient = 0; S.yaw = 0; S.yawRef = 0; S.cuad = 0;
        return;
      }
    }
    S.hab = false; S.loco = 0;
    if (S.estadoPi === "corriendo") { S.estadoPi = "esperando"; S.mensaje = "Run stopped"; }
  }

  // ------------------------------------------------------------ manual control
  async function manual(letra, extra) {
    if (S.correr) return { status: 409, body: { error: "a run is in progress: FINISHED first" } };
    S.hab = true;
    if (letra === "L" || letra === "R") { S.tarea = letra; await girarA(DIRS[(S.orient + (letra === "R" ? 1 : 3)) % 4]); S.tarea = null; }
    else if (letra === "0") { S.tarea = null; S.loco = 0; S.obj = { DI: 0, DD: 0, TI: 0, TD: 0 }; }
    else if (letra === "tirada") {
      const vx = extra.vx, ms = extra.ms || 5000;
      S.tarea = "tirada"; S.loco = 6; S.ultimoCmd = "#t=" + vx + ",0,0," + ms; S.tCmd = Date.now();
      const t0 = performance.now(); const d = DIRS[S.orient]; let v = 0;
      while (performance.now() - t0 < ms && S.tarea === "tirada") {
        const dist = distancia(vx > 0 ? "N" : "S");
        if (dist < 90) break;
        v += ((vx / 255) * 200 - v) * 0.15;
        S.obj = { DI: v, DD: v, TI: v, TD: v };
        S.cm += v * 0.1 / 60 * Math.PI * 0.0655 * 100;
        if (Math.abs(S.cm) >= CELDA_CM) {
          const s = Math.sign(S.cm);
          S.celda = [S.celda[0] + DX[d] * s, S.celda[1] + DY[d] * s]; S.cm -= CELDA_CM * s; S.celdas++;
          S.traza.push([...S.celda]);
        }
        await espera(100);
      }
      S.loco = 0; S.obj = { DI: 0, DD: 0, TI: 0, TD: 0 }; S.tarea = null;
    }
    return { status: 200, body: { ok: true } };
  }

  // ------------------------------------------------------------ telemetry at 10 Hz
  setInterval(() => {
    S.tMs += 100;
    const dists = [0, 1, 2, 3].map(i => distancia(DIRS[i]));
    const pared = dists.map(d => d < UMBRAL);
    const haySalida = pared.some(p => !p);
    for (let s = 0; s < 5; s++) pasoNeuro(dists, haySalida);
    // speed loop: rpm follows the target, pwm ~ take-off + k*rpm
    for (const r of ["DI", "DD", "TI", "TD"]) {
      const o = S.obj[r];
      S.ruedas[r] += (o - S.ruedas[r]) * 0.35 + (o ? (Math.random() - 0.5) * 6 : 0);
      if (Math.abs(S.ruedas[r]) < 0.5) S.ruedas[r] = 0;
      const p = o === 0 ? 0 : Math.min(255, 62 + Math.abs(S.ruedas[r]) * 1.15 + (r === "DD" ? -14 : 0) + (Math.random() - 0.5) * 4);
      S.pwm[r] += (p - S.pwm[r]) * 0.5;
    }
    // heading: small corrected drift
    if (S.loco === 1 || S.loco === 6) { S.yaw += (Math.random() - 0.5) * 0.35; S.yaw += (S.yawRef - S.yaw) * 0.08; }
  }, 100);

  function estado() {
    const t = {
      t_ms: S.tMs,
      dN: distancia("N"), dE: distancia("E"), dS: distancia("S"), dO: distancia("O"),
      nr0: z[0], nr1: z[1], nr2: z[2], nr3: z[3],
      mr0: z[4], mr1: z[5], mr2: z[6], mr3: z[7],
      ret: z[9], mem: z[8],
      roll: (Math.random() - 0.5) * 0.6, pitch: (Math.random() - 0.5) * 0.6,
      yaw: S.yaw, yaw_ref: S.yawRef, corr: (S.yawRef - S.yaw) * 3.6, cuad: S.cuad,
      ecen: (Math.random() - 0.5) * 8,
      cm_tramo: S.cm, celdas: S.celdas,
      estado: S.loco, cmd: S.cmd, ev: S.ev, err_giro: (S.yaw - S.yawRef), imu_ok: 1, hab: S.hab ? 1 : 0,
      vivas: 15, cortadas: 0, ciclo_ms: 8 + Math.round(Math.random() * 6),
    };
    t.pN = t.dN < UMBRAL ? 1 : 0; t.pE = t.dE < UMBRAL ? 1 : 0; t.pS = t.dS < UMBRAL ? 1 : 0; t.pO = t.dO < UMBRAL ? 1 : 0;
    const ruedas = [];
    for (const r of ["DI", "DD", "TI", "TD"]) {
      t["rpm_" + r] = S.ruedas[r]; t["obj_" + r] = S.obj[r]; t["pwm_" + r] = S.pwm[r];
      t["desp_" + r] = { DI: 152, DD: 138, TI: 160, TD: 149 }[r]; t["agc_" + r] = { DI: 88, DD: 96, TI: 74, TD: 127 }[r];
      ruedas.push({ nombre: r, rpm: S.ruedas[r], objetivo: S.obj[r], pwm: S.pwm[r], despegue: t["desp_" + r], agc: t["agc_" + r], viva: true, cortada: false });
    }
    return {
      telem: t, ruedas,
      corrida: {
        n: S.corridaN, paso: S.paso, celda: S.celda, orientacion: S.orient, heading: DIRS[S.orient],
        evento: { 0: "", 2: "nack", 3: "fin_celda", 7: "fin_giro" }[S.ev] || "", motivo: S.decision.motivo || "",
        figura: S.cam.ultima || "", en_meta: false, modo: S.corridas ? "explotacion" : "exploracion",
        temperatura: Math.max(0.1, 0.6 - 0.06 * S.corridas), corridas: S.corridas, exitos: S.exitos,
        recompensa: S.paso ? -0.05 + (Math.random() * 0.5 - 0.2) : null,
      },
      mapa: S.mapa, traza: S.traza.slice(-6),
      estado_pi: S.estadoPi, mensaje: S.mensaje, avisos: S.avisos.slice(-12),
      lado: LADO, meta: [15, 15], correr: S.correr, camara_puerto: 8081,
      celda_cm: CELDA_CM,
      neuro: S.neuro,
      decision: S.decision,
      ultimo_cmd: S.ultimoCmd, hace_cmd: Math.round((Date.now() - S.tCmd) / 100) / 10,
      estado_esp: { 0: "idle", 1: "avanzando", 2: "girando", 4: "bloqueado", 6: "manual" }[S.loco],
      params: { kp: 0.4, ki: 0.1, kd: 0, kpy: 3.6, kiy: 0, kdy: 0.7, corrmax: 30, umbralN: 200, umbralE: 200, umbralS: 200, umbralO: 200, celda: CELDA_CM, odom: 1.09, crucero: 0.12, vgiro: 60, tolgiro: 4.4, tolalin: 4.0, tolavance: 12 },
      registro: null, fw: "sim-2026-09-15",
      t: Date.now() / 1000,
    };
  }

  function datosCamara() {
    const d = { activa: true, lecturas: S.paso * 3, ultima_figura: S.cam.ultima, dist_frontal: distancia("N"), dist_decision: 160,
      area_decision: 0.06, confianza_minima: {}, figura: S.cam.figura, avistada: S.cam.avistada, alarma: !!S.cam.avistada };
    if (S.cam.figura) {
      d.confianza = S.cam.confianza; d.area_frac = S.cam.area; d.vertices = { triangulo_izq: 3, triangulo_der: 3, circulo: 8, cuadrado: 4 }[S.cam.figura];
      d.puerta_area = S.cam.area >= 0.06; d.puerta_conf = true; d.puerta_dist = d.dist_frontal <= 160; d.decidiria = d.puerta_area && d.puerta_dist;
    }
    return d;
  }

  // ------------------------------------------------------------ fake fetch
  const respuesta = (status, body) => Promise.resolve(new Response(JSON.stringify(body), { status, headers: { "Content-Type": "application/json" } }));
  const fetchReal = window.fetch.bind(window);
  window.fetch = function (url, opts) {
    const u = String(url);
    if (u.endsWith("/estado")) return respuesta(200, estado());
    if (u.endsWith("/datos")) return respuesta(200, datosCamara());
    if (u.endsWith("/orden")) {
      const c = JSON.parse((opts && opts.body) || "{}");
      switch (c.accion) {
        case "start":
          if (!S.correr) { S.correr = true; bucleCorrida(); }
          return respuesta(200, { ok: true });
        case "paro":
          S.correr = false; S.tarea = null; S.hab = false; S.loco = 0; S.obj = { DI: 0, DD: 0, TI: 0, TD: 0 };
          S.estadoPi = "esperando"; S.mensaje = "STOP from the interface: motors off (START turns them on)";
          return respuesta(200, { ok: true });
        case "final":
          S.correr = false; S.tarea = null; S.hab = false; S.loco = 0; S.obj = { DI: 0, DD: 0, TI: 0, TD: 0 };
          S.exitos++; S.corridas++; S.estadoPi = "terminado"; S.mensaje = "END declared by hand: robot braked, motors off";
          return respuesta(200, { ok: true });
        case "finished":
          S.correr = false; return respuesta(200, { ok: true });
        case "manual":
          return manual(c.letra).then(r => respuesta(r.status, r.body));
        case "manual_pwm":
          if (c.vx === 0 && c.vy === 0 && c.w === 0) return manual("0").then(r => respuesta(r.status, r.body));
          return manual("tirada", c).then(r => respuesta(r.status, r.body));
        case "banco": S.avisos.push("OK: bench " + c.letra); return respuesta(200, { ok: true });
        default: return respuesta(400, { error: "unknown action in the simulator: " + c.accion });
      }
    }
    if (u.endsWith("/parametros")) return respuesta(200, { definicion: [], valores: {} });
    return fetchReal(url, opts);
  };
  window.SIMULADOR = { S, lab, nuevoLaberinto: () => { lab = generarLaberinto(); S.mapa = {}; S.traza = []; } };
  console.info("[simulator] active: the page runs with fictitious data");
})();

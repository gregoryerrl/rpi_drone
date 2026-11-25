"use strict";

const byId = id => document.getElementById(id);
const video = byId("video");
const picture = video.getContext("2d");
const horizon = byId("horizon").getContext("2d");
const axes = ["pitch", "roll"];
const positions = ["Low", "Middle", "High"];
const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const degrees = (n, places = 0) =>
  (n > 0 ? "+" : n < 0 ? "−" : "") + Math.abs(n).toFixed(places) + "°";
let attitude = null;
let telemetry = null;
let generation = 0;
let lastFrame = -1;
let lastDraw = 0;
let videoLive = false;
let crop = {x: 0.5, y: 0.5, span: 1};

function badge(id, label, live) {
  const chip = byId(id);
  chip.textContent = label;
  chip.classList.toggle("on", live);
}

function link(id, text, online) {
  byId(id + "State").textContent = text;
  byId(id + "Dot").classList.toggle("on", online);
}

function line(ctx, x1, y1, x2, y2) {
  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(x2, y2);
  ctx.stroke();
}

function angleLayer(pitch, roll, draw) {
  horizon.save();
  // Positive nose-up moves the apparent horizon down; right bank tilts it left.
  horizon.rotate(-roll * Math.PI / 180);
  horizon.translate(0, pitch * 1.12);
  draw();
  horizon.restore();
}

function drawHorizon() {
  const ref = {};
  for (const axis of axes) {
    ref[axis] = Number(byId(axis).value);
    byId(axis + "Value").textContent = degrees(ref[axis]);
  }
  horizon.setTransform(2, 0, 0, 2, 300, 125);
  horizon.fillStyle = "#253739";
  horizon.fillRect(-150, -62.5, 300, 125);
  if (attitude) {
    angleLayer(attitude.pitch, attitude.roll, () => {
      horizon.fillStyle = "#3b5868";
      horizon.fillRect(-350, -350, 700, 350);
      horizon.fillStyle = "#405049";
      horizon.fillRect(-350, 0, 700, 350);
      horizon.strokeStyle = "#f3faf7";
      horizon.lineWidth = 1.5;
      line(horizon, -350, 0, 350, 0);
      horizon.strokeStyle = "#b0c5c0";
      horizon.lineWidth = 0.75;
      for (let d = -60; d <= 60; d += 10) {
        if (d !== 0) {
          const width = d % 20 === 0 ? 17 : 9;
          line(horizon, -width, d * 1.12, width, d * 1.12);
        }
      }
    });
  }
  angleLayer(ref.pitch, ref.roll, () => {
    horizon.strokeStyle = "#ffd17a";
    horizon.lineWidth = 2;
    horizon.setLineDash([5, 4]);
    line(horizon, -91, 0, -15, 0);
    line(horizon, 15, 0, 91, 0);
  });
  horizon.strokeStyle = "#fff";
  horizon.lineWidth = 2;
  line(horizon, -10, 0, -3, 0);
  line(horizon, 3, 0, 10, 0);
  line(horizon, 0, 1, 0, 6);
}

function trackingStatus() {
  const tracking = telemetry?.tracking;
  const mode = tracking?.mode || "Waiting";
  if (byId("trackMode").textContent !== mode)
    byId("trackMode").textContent = mode;
  byId("trackMode").classList.toggle("on", mode === "Following");
  byId("trackHint").textContent = tracking?.hint || "Waiting for the Pi.";
  for (const pos of positions)
    byId(pos.toLowerCase()).classList.toggle("active", tracking?.switch === pos);
  // This value lives only in the browser. The flight controller is opened read-only.
  byId("sim").textContent = videoLive && mode === "Following" ? "15%" : "0%";
}

function showTelemetry(data) {
  telemetry = data;
  const radio = data?.radio;
  const fcLive = typeof radio?.fc_armed === "boolean";
  const rcLive = data?.rc_live === true;
  link("pi", data ? "Online" : "Offline", Boolean(data));
  link("fc", fcLive ? (radio.fc_armed ? "Armed" : "Disarmed") : "Waiting", fcLive);
  byId("fcState").title = fcLive ? radio.fc_mode || "" : radio?.status || "";
  link("rc", rcLive ? radio.channels.length + " channels" : "Waiting", rcLive);
  byId("rcState").title = radio?.status || "";
  const channels = new Map(rcLive ? radio.channels.map(c => [c.number, c.value]) : []);
  for (const n of [1, 2, 3, 8])
    byId("ch" + n).textContent = channels.has(n) ? channels.get(n) + " µs" : "—";
  attitude = radio?.attitude_deg || null;
  badge("imuState", attitude ? "Live" : "Waiting", Boolean(attitude));
  for (const axis of axes)
    byId(axis + "Read").textContent = attitude ? degrees(attitude[axis], 1) : "—";
  drawHorizon();
  trackingStatus();
}

function drawFrame(image, state) {
  const w = video.width, h = video.height;
  const now = performance.now();
  const smooth = 1 - Math.exp(-Math.min((now - lastDraw) / 1000, 0.25) * 9);
  const box = state.box;
  if (state.mode === "Following" && box) {
    crop.span += (0.66 - crop.span) * smooth;
    const half = crop.span / 2;
    crop.x = clamp(crop.x + (box[0] - crop.x) * smooth, half, 1 - half);
    crop.y = clamp(crop.y + (box[1] - crop.y) * smooth, half, 1 - half);
  } else if (state.mode !== "Holding") {
    crop = {x: 0.5, y: 0.5, span: 1};
  }
  const left = crop.x - crop.span / 2, top = crop.y - crop.span / 2;
  picture.drawImage(image, left * image.width, top * image.height,
    crop.span * image.width, crop.span * image.height, 0, 0, w, h);

  picture.save();
  picture.shadowColor = "#071817";
  picture.shadowBlur = 3;
  picture.strokeStyle = "rgba(255,255,255,.95)";
  picture.lineWidth = 1.5;
  line(picture, w/2 - 12, h/2, w/2 - 4, h/2);
  line(picture, w/2 + 4, h/2, w/2 + 12, h/2);
  line(picture, w/2, h/2 - 12, w/2, h/2 - 4);
  line(picture, w/2, h/2 + 4, w/2, h/2 + 12);
  if (box) {
    const bw = box[2] * w / crop.span, bh = box[3] * h / crop.span;
    const x = (box[0] - left) * w / crop.span - bw / 2;
    const y = (box[1] - top) * h / crop.span - bh / 2;
    picture.strokeStyle = state.mode === "Following" ? "#80ffd1" : "#ffcf71";
    picture.lineWidth = 2.5;
    picture.strokeRect(x, y, bw, bh);
  }
  picture.restore();
  lastDraw = now;
  lastFrame = state.frame;
  videoLive = true;
  byId("placeholder").hidden = true;
  badge("camState", "Live", true);
  trackingStatus();
}

function clearVideo() {
  videoLive = false;
  lastFrame = -1;
  crop = {x: 0.5, y: 0.5, span: 1};
  picture.clearRect(0, 0, video.width, video.height);
  byId("placeholder").textContent = "Waiting for camera…";
  byId("placeholder").hidden = false;
  badge("camState", "Waiting", false);
  trackingStatus();
}

const active = id => id === generation && !document.hidden;

async function pollTelemetry(id) {
  if (!active(id)) return;
  try {
    const response = await fetch("/api/state", {signal: AbortSignal.timeout(1300)});
    if (!response.ok) throw Error("Telemetry unavailable");
    const state = await response.json();
    if (active(id)) showTelemetry(state);
  } catch {
    if (active(id)) showTelemetry(null);
  }
  if (active(id)) setTimeout(() => pollTelemetry(id), 150);
}

async function pollFrame(id) {
  if (!active(id)) return;
  const start = performance.now();
  let delay = 65;
  try {
    const response = await fetch("/frame.jpg?after=" + lastFrame, {signal: AbortSignal.timeout(1300)});
    if (!response.ok) throw Error("Camera unavailable");
    if (response.status === 204) {
      if (active(id) && start - lastDraw > 600) clearVideo();
    } else {
      const metadata = JSON.parse(response.headers.get("X-Jig-Frame"));
      const image = await createImageBitmap(await response.blob());
      try {
        if (active(id)) drawFrame(image, metadata);
      } finally {
        image.close();
      }
    }
  } catch {
    if (active(id)) clearVideo();
    delay = 600;
  }
  if (active(id)) setTimeout(() => pollFrame(id), Math.max(0, delay - (performance.now() - start)));
}

function visibilityChanged() {
  const id = ++generation;
  showTelemetry(null);
  clearVideo();
  if (!document.hidden) {
    pollTelemetry(id);
    pollFrame(id);
  }
}

for (const axis of axes) byId(axis).addEventListener("input", drawHorizon);
byId("level").addEventListener("click", () => {
  for (const axis of axes) byId(axis).value = "0";
  drawHorizon();
});
document.addEventListener("visibilitychange", visibilityChanged);
visibilityChanged();

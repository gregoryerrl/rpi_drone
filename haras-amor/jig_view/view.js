"use strict";

const byId = id => document.getElementById(id);
const video = byId("video");
const picture = video.getContext("2d");
const horizon = byId("horizon").getContext("2d");
const axes = ["pitch", "roll"];
const degrees = (n, places = 0) =>
  (n > 0 ? "+" : n < 0 ? "−" : "") + Math.abs(n).toFixed(places) + "°";
let attitude = null;
let generation = 0;
let lastFrame = -1;
let lastDraw = 0;
let lastBatteryDisplay;

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

const batteryUnits = {
  voltage_v: [2, " V"], current_a: [2, " A"], power_w: [1, " W"], remaining_pct: [0, "%"]
};

function showBatteries(reported, fcLive) {
  const packs = fcLive && Array.isArray(reported) ? reported : [];
  const display = JSON.stringify([fcLive, packs]);
  if (display === lastBatteryDisplay) return;
  lastBatteryDisplay = display;
  const warning = packs.some(pack => pack.charge_state && !["OK", "Charging"].includes(pack.charge_state));
  let status = fcLive ? "Not reported" : "Waiting for FC";
  if (packs.length === 1) status = packs[0].charge_state || "Live";
  if (packs.length > 1) status = warning ? "Battery warning" : "Live";
  badge("batteryState", status, packs.length > 0 && !warning);
  byId("batteryState").classList.toggle("warning", warning);

  const rows = document.createDocumentFragment();
  for (const pack of packs.length ? packs : [{}]) {
    const row = byId("batteryTemplate").content.cloneNode(true);
    if (packs.length > 1) {
      const name = row.querySelector(".battery-name");
      name.hidden = false;
      const sharedId = packs.some(other => other.id === pack.id && other.component !== pack.component);
      name.textContent = "Battery ID " + pack.id + (sharedId ? " · Component " + pack.component : "")
        + (pack.charge_state ? " · " + pack.charge_state : "");
    }
    for (const cell of row.querySelectorAll("[data-field]")) {
      const key = cell.dataset.field;
      const [places, unit] = batteryUnits[key];
      if (Number.isFinite(pack[key])) cell.textContent = pack[key].toFixed(places) + unit;
    }
    rows.append(row);
  }
  byId("batteryRows").replaceChildren(rows);
}

function displayRadioMeter(status, detail, active = false, stopped = false) {
  const chip = byId("radioTestStatus");
  if (chip.textContent !== status) chip.textContent = status;
  chip.classList.toggle("on", active);
  chip.classList.toggle("warning", stopped);
  byId("radioPercent").textContent = active ? "15%" : "0%";
  byId("radioMeter").value = active ? 15 : 0;
  byId("radioTestDetail").textContent = detail;
}

function showRadioTest(radio, rcLive) {
  if (typeof radio?.fc_armed !== "boolean")
    return displayRadioMeter("Waiting for FC", "Waiting for the flight controller heartbeat.");
  if (!rcLive)
    return displayRadioMeter("Waiting for radio", "Waiting for fresh TX16S input.");
  const trace = radio.switch_trace || [];
  const latest = trace.at(-1);
  if (!latest)
    return displayRadioMeter("Waiting for switches", "Both CH6 and CH8 need a valid reading.");
  if (latest.ch6 === "high")
    return displayRadioMeter("CH6 stop", "The Pi received the kill switch.", false, true);
  if (latest.ch6 !== "lower")
    return displayRadioMeter("Check CH6", "CH6 is between switch positions.", false, true);
  if (latest.ch8 === "lower")
    return displayRadioMeter("Ready", "Move CH8 high to start the meter.");
  if (latest.ch8 !== "high")
    return displayRadioMeter("Reading CH8", "CH8 is between switch positions.");

  // Reconstruct the display from receiver observations. A browser can join
  // mid-test, and a momentary CH6 pulse still appears in the history.
  const lower = trace.findLastIndex(sample => sample.ch8 === "lower");
  if (lower < 0 || trace[lower].ch6 !== "lower")
    return displayRadioMeter("Cycle CH8", "Move CH8 low or middle with CH6 clear, then high.");
  const startIndex = trace.findIndex((sample, i) => i > lower && sample.ch8 === "high");
  const start = trace[startIndex].at;
  if (trace.slice(startIndex).some(sample => sample.ch6 !== "lower" && sample.at - start < 10))
    return displayRadioMeter("Canceled by CH6", "Cycle CH8 to start a new test.", false, true);
  const remaining = 10 - (radio.received_at - start);
  if (remaining <= 0)
    return displayRadioMeter("Complete", "Ten seconds elapsed. Cycle CH8 for another test.");
  displayRadioMeter("Timing", remaining.toFixed(1) + " s remaining.", true);
}

function showTelemetry(data) {
  const radio = data?.radio;
  const fcLive = typeof radio?.fc_armed === "boolean";
  showBatteries(radio?.batteries, fcLive);
  const rcLive = data?.rc_live === true;
  showRadioTest(radio, rcLive);
  link("pi", data ? "Online" : "Offline", Boolean(data));
  link("fc", fcLive ? (radio.fc_armed ? "Armed" : "Disarmed") : "Waiting", fcLive);
  byId("fcState").title = fcLive ? radio.fc_mode || "" : radio?.status || "";
  link("rc", rcLive ? radio.channels.length + " channels" : "Waiting", rcLive);
  byId("rcState").title = radio?.status || "";
  const channels = new Map(rcLive ? radio.channels.map(c => [c.number, c.value]) : []);
  for (const n of [1, 2, 3, 5, 8])
    byId("ch" + n).textContent = channels.has(n) ? channels.get(n) + " µs" : "—";
  const killPulse = channels.get(6);
  const killHigh = killPulse >= 1850 && killPulse <= 2200;
  byId("ch6").textContent = killPulse >= 800 && killPulse <= 2200
    ? killPulse + " µs" + (killHigh ? " · STOP" : "") : "—";
  attitude = radio?.attitude_deg || null;
  badge("imuState", attitude ? "Live" : "Waiting", Boolean(attitude));
  for (const axis of axes)
    byId(axis + "Read").textContent = attitude ? degrees(attitude[axis], 1) : "—";
  drawHorizon();
}

function drawFrame(image, number) {
  picture.drawImage(image, 0, 0, video.width, video.height);
  lastDraw = performance.now();
  lastFrame = number;
  byId("placeholder").hidden = true;
  badge("camState", "Live", true);
}

function clearVideo() {
  lastFrame = -1;
  picture.clearRect(0, 0, video.width, video.height);
  byId("placeholder").textContent = "Waiting for camera…";
  byId("placeholder").hidden = false;
  badge("camState", "Waiting", false);
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
        if (active(id)) drawFrame(image, metadata.frame);
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

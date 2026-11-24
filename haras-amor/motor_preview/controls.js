"use strict";
const el = id => document.getElementById(id);
const slider = el("throttle"), statusLine = el("status"), reconnect = el("reconnect");
const canControl = document.documentElement.dataset.operator === "yes";
const idleText = "PX4 disarmed. Hold and drag the slider. Release to stop.";
let socket = null, ready = false, gesture = null, sequence = 0, pollTimer = null;

function showLevel() {
  if (!gesture) slider.value = "0";
  el("value").value = slider.value + "%";
}

function release() {
  const old = gesture;
  gesture = null;
  clearTimeout(pollTimer);
  showLevel();
  if (old && socket?.readyState === WebSocket.OPEN) {
    socket.send(JSON.stringify({release: old.id}));
    statusLine.textContent = "Releasing motors…";
  }
}

function disconnect(reason) {
  release();
  const old = socket;
  socket = null;
  ready = false;
  slider.disabled = true;
  if (old) old.close();
  statusLine.textContent = reason;
  reconnect.hidden = !canControl;
}

function expectPoll() {
  clearTimeout(pollTimer);
  pollTimer = setTimeout(() => disconnect("Controller stopped responding. Reconnect to start a new hold."), 850);
}

function connect() {
  if (!canControl || socket || document.hidden) return;
  reconnect.hidden = true;
  statusLine.textContent = "Checking the flight controller…";
  const ws = new WebSocket("ws://" + location.host + "/motors");
  socket = ws;
  ws.onmessage = event => {
    if (socket !== ws) return;
    const msg = JSON.parse(event.data);
    if (msg.error) {
      disconnect(msg.error);
    } else if (msg.ready) {
      ready = true;
      slider.disabled = false;
      statusLine.textContent = idleText;
    } else if (msg.poll && msg.hold === gesture?.id) {
      if (document.hidden || !document.hasFocus()) { release(); return; }
      expectPoll();
      ws.send(JSON.stringify({hold: gesture.id, poll: msg.poll, percent: Number(slider.value)}));
    } else if (msg.released && !gesture) {
      statusLine.textContent = idleText;
    }
  };
  ws.onclose = () => {
    if (socket === ws) disconnect("Controller disconnected. Reconnect to start a new hold.");
  };
  ws.onerror = () => {
    if (socket === ws) disconnect("Controller connection unavailable.");
  };
}

function start(kind, key) {
  if (!ready || gesture || document.hidden || socket?.readyState !== WebSocket.OPEN) return false;
  gesture = {id: ++sequence, kind, key};
  expectPoll();
  socket.send(JSON.stringify({hold: gesture.id}));
  statusLine.textContent = "Release to return all four motors to zero.";
  return true;
}

slider.addEventListener("pointerdown", event => {
  if (event.button === 0 && event.isPrimary && start("pointer", event.pointerId)) {
    try { slider.setPointerCapture(event.pointerId); }
    catch { release(); }
  }
});
function pointerEnd(event) {
  if (gesture?.kind === "pointer" && event.pointerId === gesture.key) release();
}
window.addEventListener("pointerup", pointerEnd, true);
window.addEventListener("pointercancel", pointerEnd, true);
slider.addEventListener("lostpointercapture", pointerEnd);
window.addEventListener("pointermove", event => { if (event.buttons === 0) pointerEnd(event); });
const adjustKeys = new Set(["ArrowLeft","ArrowRight","ArrowUp","ArrowDown","Home","End","PageUp","PageDown"]);
slider.addEventListener("keydown", event => {
  if (adjustKeys.has(event.key) && !event.repeat) start("key", event.key);
});
window.addEventListener("keyup", event => {
  if (gesture?.kind === "key" && gesture.key === event.key) release();
}, true);
slider.addEventListener("input", showLevel);
slider.addEventListener("change", showLevel);
slider.addEventListener("blur", release);
window.addEventListener("blur", release);
window.addEventListener("pagehide", () => disconnect("Preview closed."));
window.addEventListener("offline", () => disconnect("Network disconnected."));
document.addEventListener("visibilitychange", () => {
  if (document.hidden && canControl) disconnect("Controller paused while the page was hidden.");
});
reconnect.addEventListener("click", connect);
el("video").addEventListener("error", () => { el("video-error").hidden = false; });
showLevel();
if (canControl) {
  connect();
} else {
  statusLine.textContent = "Camera preview only. Motor control is available through SSH.";
  el("ssh").hidden = false;
  const user = document.documentElement.dataset.user;
  el("tunnel").textContent = "ssh -N -L 127.0.0.1:8080:127.0.0.1:" +
      (location.port || "80") + " " + user + "@" + location.hostname;
}

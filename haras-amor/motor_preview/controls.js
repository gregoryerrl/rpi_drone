"use strict";
const el = id => document.getElementById(id);
const slider = el("throttle"), statusLine = el("status"), reconnect = el("reconnect");
const canControl = document.documentElement.dataset.operator === "yes";
const idleText = "PX4 disarmed. Hold and drag the slider. Release to stop.";
let socket = null, ready = false, gesture = null, sequence = 0, pollTimer = null;
const stopButton = el("motor-stop"), resetButton = el("reset-stop"), stopStatus = el("stop-status");
let latched = null, stopToken = null, actionId = 0, changingStop = false, needStopConfirmation = false;

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
    try { socket.send(JSON.stringify({release: old.id})); } catch {}
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

function applyStopState(state) {
  if (needStopConfirmation && !state.stopped) return;
  if (state.stopped) {
    needStopConfirmation = false;
    release();
  }
  latched = state.stopped;
  stopToken = state.token || null;
  slider.disabled = !ready || latched;
  resetButton.hidden = !canControl || !latched || !stopToken;
  const note = state.storage_error ? "Motor stop latched in memory. Could not verify it on disk." :
      latched ? "Bench motor stop latched." : "";
  if (stopStatus.textContent !== note) stopStatus.textContent = note;
}

async function refreshStopState() {
  if (changingStop) return;
  const id = actionId;
  try {
    const r = await fetch("/api/motor-stop", {cache:"no-store", signal:AbortSignal.timeout(1200)});
    if (r.ok) {
      const state = await r.json();
      if (id === actionId && !changingStop) applyStopState(state);
    }
  } catch {}  // The next poll retries. The stop button also works without a WebSocket.
}

async function changeStop(reset) {
  if (!canControl || (reset && (!stopToken || changingStop))) return;
  const token = stopToken, id = ++actionId;
  changingStop = true;
  latched = true;
  if (!reset) needStopConfirmation = true;
  slider.disabled = true;
  resetButton.hidden = true;
  release();
  stopStatus.textContent = reset ? "Resetting motor stop…" : "Sending motor stop…";
  try {
    const options = {method:"POST", cache:"no-store", signal:AbortSignal.timeout(2000)};
    if (reset) {
      options.headers = {"Content-Type":"application/json"};
      options.body = JSON.stringify({token});
    }
    const r = await fetch(reset ? "/api/motor-stop/reset" : "/api/motor-stop", options);
    if (!r.ok) throw new Error("Stop state unavailable");
    const state = await r.json();
    if (id !== actionId) return;
    applyStopState(state);
    if (reset && !state.stopped) {
      disconnect("Motor stop reset.");
      connect();  // A new connection starts at zero and needs a new physical press.
    }
  } catch {
    if (id !== actionId) return;
    disconnect(reset ? "Reset unconfirmed." : "Motor control disconnected.");
    stopStatus.textContent = reset ? "Reset unconfirmed. Check the stop state." :
        "Stop unconfirmed. Use the independent kill switch or motor-power cutoff.";
  } finally {
    if (id === actionId) {
      changingStop = false;
      refreshStopState();
    }
  }
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
    } else if (msg.stopped) {
      ready = false;
      if (!changingStop) ++actionId;
      applyStopState({stopped:true, token:stopToken});
      statusLine.textContent = "Motor output blocked. Sensor preview available.";
      refreshStopState();
    } else if (msg.ready) {
      ready = true;
      slider.disabled = latched !== false;
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
  if (!ready || latched !== false || gesture || document.hidden || socket?.readyState !== WebSocket.OPEN) return false;
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
stopButton.hidden = !canControl;
stopButton.addEventListener("click", () => changeStop(false));
resetButton.addEventListener("click", () => changeStop(true));
window.addEventListener("keydown", event => {
  if (canControl && event.key === "Escape" && !event.repeat) {
    event.preventDefault();
    changeStop(false);
  }
}, true);
async function watchStopState() {
  if (!document.hidden) await refreshStopState();
  window.setTimeout(watchStopState, 1000);
}
watchStopState();
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

"use strict";
const el = id => document.getElementById(id);
const slider = el("throttle"), statusLine = el("status"), reconnect = el("reconnect");
const canControl = document.documentElement.dataset.operator === "yes";
const idleText = "PX4 disarmed. Hold and drag the slider. Release to stop.";
let socket = null, ready = false, gesture = null, sequence = 0, pollTimer = null;
const stopButton = el("motor-stop"), resetButton = el("reset-stop"), stopStatus = el("stop-status");
let latched = null, stopToken = null, actionId = 0, changingStop = false, needStopConfirmation = false;
const jigSwitch = el("auto-stabilize"), armButton = el("arm-jig");
const angles = [el("target-roll"), el("target-pitch")];
let jig = false, jigReady = false, jigArmed = false, jigArming = false, endingJig = false;
let armRequest = false, confirmArmAt = 0;
let fcLevelTarget = false;
let jigStickPercent = 0;
let jigTimer = null;

function setText(element, value) {
  if (element.textContent !== value) element.textContent = value;
}

function jigStateLabel() {
  if (endingJig) return "Stopping…";
  if (!jig) return "Off";
  if (!jigReady) return "Selecting…";
  if (jigArming) return "Arming…";
  if (!jigArmed) return "Standby";
  return "Armed · " + jigStickPercent + "% stick";
}

function controls() {
  const enabled = canControl && ready && latched === false && !changingStop;
  slider.disabled = !enabled || (jig && (!jigReady || !jigArmed || endingJig));
  jigSwitch.disabled = !enabled || endingJig || (!jig && !!gesture);
  jigSwitch.setAttribute("aria-checked", String(jig && jigReady && !endingJig));
  setText(jigSwitch, "Auto stabilize · " + jigStateLabel());
  for (const input of angles) input.disabled = !enabled || !jigReady || endingJig || jigArming;
  el("target-level").disabled = angles[0].disabled;
  armButton.hidden = !enabled || !jigReady || jigArmed || endingJig;
  armButton.disabled = jigArming || armRequest || !fcLevelTarget || angles.some(a => Number(a.value) !== 0);
  el("jig-power-note").hidden = !jig;
  setText(el("power-label"), jig ? "Jig throttle stick · hold to apply, release to end the run" :
      "All four motors · propellers off");
}

function renderAngles() {
  for (const input of angles) el(input.id + "-value").value = (Number(input.value) > 0 ? "+" : "") + input.value + "°";
}
function levelTarget() {
  for (const input of angles) input.value = "0";
  renderAngles();
  controls();
}
for (const input of angles) input.addEventListener("input", () => {
  const other = angles.find(a => a !== input);
  const radians = Math.PI / 180, cap = Number(input.max);
  const limit = Math.floor(Math.acos(Math.min(1, Math.cos(cap * radians) /
      Math.cos(Number(other.value) * radians))) / radians + 0.000001);
  input.value = String(Math.max(-limit, Math.min(limit, Number(input.value))));
  confirmArmAt = 0;
  armButton.textContent = "Arm secured jig…";
  renderAngles();
  controls();
});
el("target-level").addEventListener("click", levelTarget);

function clearJig() {
  clearTimeout(jigTimer);
  jig = jigReady = jigArmed = jigArming = endingJig = armRequest = false;
  fcLevelTarget = false;
  jigStickPercent = 0;
  confirmArmAt = 0;
  armButton.textContent = "Arm secured jig…";
  el("fc-target").textContent = "";
  levelTarget();
}

function endJig() {
  if (!jig || endingJig) return;
  endingJig = true;
  armRequest = false;
  controls();
  if (socket?.readyState === WebSocket.OPEN) {
    try { socket.send(JSON.stringify({jig:"off"})); }
    catch { el("jig-status").textContent = "Jig stop unconfirmed. Use the motor-power disconnect."; }
  }
}
jigSwitch.addEventListener("click", () => {
  if (jig) { endJig(); return; }
  if (!ready || latched !== false || gesture || socket?.readyState !== WebSocket.OPEN) return;
  jig = true;
  levelTarget();
  el("jig-status").textContent = "Selecting PX4 Stabilized with zero throttle…";
  socket.send(JSON.stringify({jig:"on"}));
});
armButton.addEventListener("click", () => {
  if (!jigReady || jigArmed || jigArming || endingJig || latched !== false) return;
  const now = performance.now();
  if (!confirmArmAt || now - confirmArmAt > 7000) {
    confirmArmAt = now;
    armButton.textContent = "Confirm arm · motors may spin";
  } else if (now - confirmArmAt > 800) {
    armRequest = true;
    confirmArmAt = 0;
    armButton.textContent = "Arm secured jig…";
    controls();
  }
});

function showLevel() {
  if (!gesture) slider.value = "0";
  el("value").value = slider.value + "%";
}

function release() {
  const old = gesture;
  gesture = null;
  clearTimeout(pollTimer);
  showLevel();
  if (old?.jig) {
    endJig();
  } else if (old && socket?.readyState === WebSocket.OPEN) {
    try { socket.send(JSON.stringify({release: old.id})); } catch {}
    statusLine.textContent = "Releasing motors…";
  }
  controls();
}

function disconnect(reason) {
  release();
  const old = socket;
  socket = null;
  ready = false;
  if (jig) el("jig-status").textContent = "Jig link lost. If motors are powered, use the motor-power disconnect.";
  clearJig();
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
  controls();
  resetButton.hidden = !canControl || !latched || !stopToken;
  const note = state.storage_error ? "Motor stop latched in memory. Could not verify it on disk." :
      latched ? "Bench motor stop latched." + (state.fc_stop ? " " + state.fc_stop : "") : "";
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
      if (jig) el("jig-status").textContent = "Pi jig output stopped. Check the FC disarm status.";
      clearJig();
      if (!changingStop) ++actionId;
      applyStopState({stopped:true, token:stopToken});
      statusLine.textContent = "Motor output blocked. Sensor preview available.";
      refreshStopState();
    } else if (msg.ready) {
      ready = true;
      controls();
      statusLine.textContent = idleText;
      if (msg.jig_disarmed) el("jig-status").textContent = "Auto stabilize off. PX4 reports disarmed.";
    } else if (msg.jig_off) {
      if (gesture?.jig) { gesture = null; showLevel(); }
      clearJig();
      el("jig-status").textContent = [msg.jig_off, msg.fc_stop].filter(Boolean).join(" ");
      statusLine.textContent = msg.fc_stop || idleText;
      if (msg.fc_pending) { ready = false; controls(); }
    } else if (msg.jig_poll) {
      if (!jig || endingJig) return;
      clearTimeout(jigTimer);
      jigTimer = setTimeout(() => disconnect("Jig link lost. Use the motor-power disconnect if still powered."), 1000);
      if (document.hidden || !document.hasFocus() || latched !== false) { endJig(); return; }
      jigReady = msg.confirmed;
      jigArmed = msg.armed;
      jigArming = msg.arming;
      jigStickPercent = msg.stick_pct;
      fcLevelTarget = !!msg.fc_target && msg.fc_target.every(a => Math.abs(a) <= .4);
      let detail = "Selecting PX4 Stabilized with zero throttle…";
      if (jigArming) detail = "Waiting for PX4 to confirm or decline arming.";
      else if (jigArmed) detail = jigStickPercent === 0 ?
          "PX4 reports armed. The Pi is sending zero throttle. Motors may idle." :
          "PX4 reports armed. The percentage shows the throttle stick command. Motor speed is unmeasured.";
      else if (jigReady) detail = "PX4 is disarmed at 0% throttle. The sliders set angle targets. The motors cannot move the jig or resist a tilt.";
      setText(el("jig-status"), [detail, msg.notice].filter(Boolean).join(" "));
      let powerStatus = "Jig throttle is at zero.";
      if (jigArmed) powerStatus = "Releasing the throttle ends the run and requests disarm.";
      else if (jigReady && !jigArming && angles.some(a => Number(a.value) !== 0))
        powerStatus = "Arming requires both angle targets at 0°.";
      else if (jigReady && !jigArming && !fcLevelTarget)
        powerStatus = "Waiting for PX4 to report level angle targets.";
      setText(statusLine, powerStatus);
      setText(el("fc-target"), msg.fc_target ? "FC target · roll " + msg.fc_target[0] + "° · pitch " +
          msg.fc_target[1] + "°" : "Waiting for the FC attitude target.");
      const arm = armRequest;
      armRequest = false;
      ws.send(JSON.stringify({jig_poll:msg.jig_poll, roll:Number(angles[0].value), pitch:Number(angles[1].value),
          throttle:jigArmed && gesture?.jig ? Number(slider.value) : 0, arm}));
      controls();
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
  if (jig && (!jigReady || !jigArmed || endingJig)) return false;
  gesture = {id: ++sequence, kind, key, jig};
  if (!jig) {
    expectPoll();
    socket.send(JSON.stringify({hold: gesture.id}));
  }
  controls();
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
  if (gesture?.jig && gesture.kind === "pointer" && event.key.startsWith("Arrow")) {
    event.preventDefault();
    const axis = angles[event.key === "ArrowLeft" || event.key === "ArrowRight" ? 0 : 1];
    if (event.key === "ArrowUp" || event.key === "ArrowRight") axis.stepUp();
    else axis.stepDown();
    axis.dispatchEvent(new Event("input", {bubbles:true}));
    return;
  }
  if (adjustKeys.has(event.key) && !event.repeat) start("key", event.key);
});
window.addEventListener("keyup", event => {
  if (gesture?.kind === "key" && gesture.key === event.key) release();
}, true);
slider.addEventListener("input", showLevel);
slider.addEventListener("change", showLevel);
slider.addEventListener("blur", release);
window.addEventListener("blur", () => { release(); endJig(); });
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
  el("jig-status").textContent = "Open the SSH operator preview to set roll and pitch.";
  statusLine.textContent = "Camera preview only. Motor control is available through SSH.";
  el("ssh").hidden = false;
  const user = document.documentElement.dataset.user;
  el("tunnel").textContent = "ssh -N -L 127.0.0.1:8080:127.0.0.1:" +
      (location.port || "80") + " " + user + "@" + location.hostname;
}

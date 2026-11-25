"use strict";

// This page models switch timing with local form controls and a display meter.
const byId = id => document.getElementById(id);
const arm = byId("arm");
const timed = byId("timed");
const kill = byId("kill");
const manual = byId("manual");
let armed = false;
const runSeconds = 20;
let handoffEnds = 0;
let timedOut = false;
let killDisarms = 0;
let redraw;

function advanceClock(now) {
  if (killDisarms && now >= killDisarms) {
    armed = false;
    killDisarms = 0;
  }
  if (handoffEnds && now >= handoffEnds) {
    handoffEnds = 0;
    timedOut = true;
  }
}

function render() {
  const now = performance.now();
  advanceClock(now);
  let percent = 0;
  let source = "Disarmed";
  let detail = arm.checked ? "Cycle virtual CH5 off and on to arm again." : "Raise virtual CH5 to use the manual throttle.";
  if (kill.checked) {
    source = "Kill active";
    detail = armed
      ? "Outputs are zero. Automatic disarm in " + ((killDisarms - now) / 1000).toFixed(1) + " s."
      : "Outputs are zero. The simulated aircraft is disarmed.";
  } else if (armed && handoffEnds) {
    source = "Timed";
    percent = 25;
    detail = "All four outputs stop in " + ((handoffEnds - now) / 1000).toFixed(1) + " s.";
  } else if (armed && timedOut) {
    source = "Timed out";
    detail = "All four outputs are zero. Cycle virtual CH8 for a new run or CH5 for manual throttle.";
  } else if (armed) {
    source = "Manual";
    percent = Number(manual.value);
    detail = timed.checked ? "Cycle virtual CH8 low and high for another timed run."
                           : "Raise virtual CH8 for one timed run.";
  }
  const chip = byId("source");
  if (chip.textContent !== source) chip.textContent = source;
  chip.classList.toggle("on", armed && !kill.checked && !timedOut);
  chip.classList.toggle("warning", kill.checked);
  byId("detail").textContent = detail;
  byId("manualValue").textContent = manual.value + "%";
  byId("percent").textContent = percent + "%";
  byId("motorMeter").value = percent;
  clearTimeout(redraw);
  const deadline = killDisarms || handoffEnds;
  if (deadline) redraw = setTimeout(render, Math.min(100, Math.max(1, deadline - now)));
}

function switchChanged(event) {
  const now = performance.now();
  // Account for elapsed time even if the browser throttled a background tab.
  advanceClock(now);
  if (event.target === arm) {
    armed = arm.checked && !kill.checked;
    handoffEnds = killDisarms = 0;
    timedOut = false;
  } else if (event.target === kill) {
    handoffEnds = 0;
    killDisarms = kill.checked && armed ? now + 5000 : 0;
  } else if (timed.checked && armed && !kill.checked && !handoffEnds) {
    timedOut = false;
    handoffEnds = now + runSeconds * 1000;
  }
  render();
}

for (const control of [arm, timed, kill]) {
  control.checked = false;
  control.addEventListener("change", switchChanged);
}
manual.value = "1";
manual.addEventListener("input", render);
render();

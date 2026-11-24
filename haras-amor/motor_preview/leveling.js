"use strict";
(() => {
  const status = document.getElementById("level-status");
  const lastArm = document.getElementById("last-arm");
  const corners = document.getElementById("corners");
  const motorColumn = document.getElementById("motor-column");
  const previewBaseline = 45;
  const rows = ["roll", "pitch"].map(axis => [axis, document.getElementById("level-" + axis).cells]);
  const signed = value => (value > 0 ? "+" : "") + value.toFixed(1);
  const motorNames = ids => ids.map(n => "M" + n).join(", ");

  function render(state) {
    if (status.textContent !== state.message) status.textContent = state.message;
    if (state.last_arm_attempt) {
      const attempt = state.last_arm_attempt;
      const when = new Date(attempt.time * 1000).toLocaleString();
      const message = ["Last arm attempt · " + when, attempt.message, attempt.warning].filter(Boolean).join(" ");
      if (lastArm.textContent !== message) lastArm.textContent = message;
      lastArm.hidden = false;
    }
    const preview = state.jig?.confirmed && !state.jig.armed && !state.jig.arming;
    motorColumn.textContent = preview ? "Per-axis preview · " + previewBaseline + "% simulated baseline" : "Predicted lift increase";
    for (const [axis, cells] of rows) {
      const data = state.axes?.[axis];
      if (!data) {
        for (let i = 1; i < cells.length; ++i) cells[i].textContent = "—";
        continue;
      }
      cells[1].textContent = signed(data.angle_deg) + "°";
      cells[2].textContent = signed(data.rate_deg_s) + "°/s";
      cells[3].textContent = signed(data.correction_deg_s2) + "°/s²";
      if (data.more_lift === null) {
        cells[4].textContent = "Waiting for rotor geometry";
      } else if (preview && data.more_lift.length) {
        const lower = state.corners.map((_, i) => i + 1).filter(n => !data.more_lift.includes(n));
        cells[4].textContent = motorNames(data.more_lift) + " ↑ · " + motorNames(lower) + " ↓";
      } else if (preview) {
        cells[4].textContent = "All at " + previewBaseline + "%";
      } else {
        cells[4].textContent = motorNames(data.more_lift) || "Even";
      }
    }
    corners.textContent = (state.corners || []).map((name, i) => "M" + (i + 1) + " " + name).join(" · ");
  }

  async function refresh() {
    if (!document.hidden) {
      try {
        const response = await fetch("/api/leveling", {cache:"no-store", signal:AbortSignal.timeout(900)});
        if (!response.ok) throw new Error("Telemetry unavailable");
        render(await response.json());
      } catch {
        render({message:"Telemetry connection unavailable."});
      }
    }
    window.setTimeout(refresh, document.hidden ? 1000 : 250);
  }
  refresh();
})();

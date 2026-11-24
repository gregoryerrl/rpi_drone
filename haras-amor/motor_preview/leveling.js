"use strict";
(() => {
  const status = document.getElementById("level-status");
  const corners = document.getElementById("corners");
  const rows = ["roll", "pitch"].map(axis => [axis, document.getElementById("level-" + axis).cells]);
  const signed = value => (value > 0 ? "+" : "") + value.toFixed(1);

  function render(state) {
    if (status.textContent !== state.message) status.textContent = state.message;
    for (const [axis, cells] of rows) {
      const data = state.axes?.[axis];
      if (!data) {
        for (let i = 1; i < cells.length; ++i) cells[i].textContent = "—";
        continue;
      }
      cells[1].textContent = signed(data.angle_deg) + "°";
      cells[2].textContent = signed(data.rate_deg_s) + "°/s";
      cells[3].textContent = signed(data.correction_deg_s2) + "°/s²";
      cells[4].textContent = data.more_lift === null ? "Unknown" :
          data.more_lift.map(n => "M" + n).join(", ") || "Even";
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

// v0 viewer. Three deck.gl layers stacked on a 3D orthographic-ish view:
//   - PointCloudLayer: decimated live returns
//   - HexagonLayer:    ground-plane coverage heatmap (density only — Tier 1)
//   - PathLayer:       odometry trajectory
//
// Tier-1 is deliberately a density heatmap. The v1 adequacy metric is
// research-thread only and does not run here. See docs/METHOD_NOTES.md.
//
// Depends on vendor/deck.min.js (UMD — exposes window.deck) and
// src/ros.js (window.TARP_ROS).

(function () {
  const { Deck, OrthographicView, PointCloudLayer, PathLayer, HexagonLayer } = deck;

  const ROS_URL = `ws://${location.hostname || "localhost"}:9090`;
  const MAX_LIVE_POINTS = 50_000;
  const HEX_RADIUS_M    = 0.5;
  const TRAJ_MAX_PTS    = 5_000;
  const COVERAGE_CAP    = 500_000;

  const livePoints = new Float32Array(MAX_LIVE_POINTS * 3);
  let livePointCount = 0;
  const coverage = [];
  const trajectory = [];
  let frameCount = 0;
  let pointsThisSecond = 0;

  const deckInstance = new Deck({
    parent: document.getElementById("map"),
    views: new OrthographicView({ flipY: false }),
    initialViewState: { target: [0, 0, 0], zoom: 5 },
    controller: { doubleClickZoom: false },
    getTooltip: null,
  });

  function rebuildLayers() {
    const pts = livePoints.subarray(0, livePointCount * 3);
    deckInstance.setProps({
      layers: [
        new HexagonLayer({
          id: "coverage",
          data: coverage,
          getPosition: (d) => d,
          radius: HEX_RADIUS_M,
          elevationScale: 0,
          extruded: false,
          opacity: 0.55,
          pickable: false,
          colorRange: [
            [40,  40,  60],
            [70,  110, 130],
            [90,  170, 150],
            [130, 200, 120],
            [200, 210, 90],
            [240, 180, 60],
          ],
        }),
        new PointCloudLayer({
          id: "live-points",
          data: { length: livePointCount, attributes: { getPosition: { value: pts, size: 3 } } },
          pointSize: 2,
          getColor: [230, 240, 250, 200],
          opacity: 0.9,
        }),
        new PathLayer({
          id: "trajectory",
          data: trajectory.length ? [{ path: trajectory }] : [],
          getPath: (d) => d.path,
          getColor: [255, 120, 60, 220],
          getWidth: 2,
          widthUnits: "pixels",
        }),
      ],
    });
  }

  const statusEl = document.getElementById("status");
  const ppsEl    = document.getElementById("pps");
  const fcountEl = document.getElementById("fcount");
  const odomEl   = document.getElementById("odom");

  TARP_ROS.connect({
    url: ROS_URL,
    onStatus(s) {
      statusEl.textContent = s;
      statusEl.className = `status ${s}`;
    },
    onPoints(xyz) {
      frameCount++;
      fcountEl.textContent = String(frameCount);
      pointsThisSecond += xyz.length / 3;

      const n = xyz.length / 3;
      const toCopy = Math.min(n, MAX_LIVE_POINTS);
      const src = xyz.subarray((n - toCopy) * 3);
      livePoints.set(src, 0);
      livePointCount = toCopy;

      for (let i = 0; i < toCopy; i++) {
        coverage.push([src[i * 3], src[i * 3 + 1]]);
      }
      if (coverage.length > COVERAGE_CAP) coverage.splice(0, coverage.length - COVERAGE_CAP);

      rebuildLayers();
    },
    onOdom(o) {
      trajectory.push(o.position);
      if (trajectory.length > TRAJ_MAX_PTS) trajectory.shift();
      odomEl.textContent = `${o.position[0].toFixed(1)}, ${o.position[1].toFixed(1)}`;
      rebuildLayers();
    },
  });

  setInterval(() => {
    ppsEl.textContent = pointsThisSecond.toLocaleString();
    pointsThisSecond = 0;
  }, 1000);

  const btnRecord = document.getElementById("btn-record");
  const btnWaypoint = document.getElementById("btn-waypoint");
  const btnFlag   = document.getElementById("btn-flag");
  let recording = false;

  btnRecord.addEventListener("click", () => {
    recording = !recording;
    btnRecord.classList.toggle("recording", recording);
    btnRecord.textContent = recording ? "stop" : "record";
  });
  btnWaypoint.addEventListener("click", () => {
    console.log("[waypoint]", trajectory[trajectory.length - 1]);
  });
  btnFlag.addEventListener("click", () => {
    console.log("[flag]", trajectory[trajectory.length - 1]);
  });

  rebuildLayers();
})();

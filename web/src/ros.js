// Thin wrapper around roslibjs. Kept small on purpose — just connect + subscribe
// to the two v0 topics and hand decoded points / pose off to the caller.
//
// Depends on vendor/roslib.min.js (UMD — exposes window.ROSLIB). Loaded before
// this file in index.html.

(function (global) {
  const TOPIC_POINTS = "/tarp/points";
  const TOPIC_ODOM   = "/tarp/odom";

  function connect({ url, onStatus, onPoints, onOdom }) {
    const ros = new ROSLIB.Ros({ url });
    ros.on("connection", () => onStatus("online"));
    ros.on("close",      () => onStatus("offline"));
    ros.on("error",      () => onStatus("error"));

    const pointsTopic = new ROSLIB.Topic({
      ros,
      name: TOPIC_POINTS,
      messageType: "sensor_msgs/msg/PointCloud2",
      throttle_rate: 100,
      queue_length: 1,
    });
    pointsTopic.subscribe((msg) => onPoints(decodePointCloud2(msg)));

    const odomTopic = new ROSLIB.Topic({
      ros,
      name: TOPIC_ODOM,
      messageType: "nav_msgs/msg/Odometry",
    });
    odomTopic.subscribe((msg) => onOdom(decodeOdom(msg)));

    return ros;
  }

  function decodePointCloud2(msg) {
    const raw  = atob(msg.data);
    const buf  = new ArrayBuffer(raw.length);
    const view = new Uint8Array(buf);
    for (let i = 0; i < raw.length; i++) view[i] = raw.charCodeAt(i);
    const step = msg.point_step || 12;
    const n = msg.width * msg.height;
    const out = new Float32Array(n * 3);
    const dv = new DataView(buf);
    for (let i = 0; i < n; i++) {
      const off = i * step;
      out[i * 3]     = dv.getFloat32(off,     true);
      out[i * 3 + 1] = dv.getFloat32(off + 4, true);
      out[i * 3 + 2] = dv.getFloat32(off + 8, true);
    }
    return out;
  }

  function decodeOdom(msg) {
    const p = msg.pose.pose.position;
    const q = msg.pose.pose.orientation;
    return {
      position: [p.x, p.y, p.z],
      orientation: [q.x, q.y, q.z, q.w],
      stamp: msg.header.stamp,
    };
  }

  global.TARP_ROS = { connect };
})(window);

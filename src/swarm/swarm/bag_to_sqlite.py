#!/usr/bin/env python3
"""
bag_to_sqlite.py — Convert a ROS 2 bag to a SQLite database for analysis.

Usage:
    python3 bag_to_sqlite.py <bag_path> [output.db]

Each topic becomes its own table.  Message fields are flattened into typed
columns.  Every row carries:
    timestamp_ns  INTEGER  — receive timestamp in nanoseconds (bag clock)
    timestamp_s   REAL     — same value in seconds (convenient for plotting)

Supported message types (fully flattened):
    geometry_msgs/msg/Twist        → linear_{x,y,z}, angular_{x,y,z}
    nav_msgs/msg/Odometry          → pose_{x,y,z,qx,qy,qz,qw},
                                     vel_lin_{x,y,z}, vel_ang_{x,y,z}
    sensor_msgs/msg/Imu            → ori_{x,y,z,w}, ang_vel_{x,y,z},
                                     lin_acc_{x,y,z}
    sensor_msgs/msg/JointState     → name_json, pos_json, vel_json, eff_json
    sensor_msgs/msg/Image          → width, height, encoding, step, data BLOB
    sensor_msgs/msg/PointCloud2    → width, height, point_step, data BLOB
    std_msgs/msg/Bool              → data INTEGER (0/1)
    std_msgs/msg/Float64           → data REAL
    std_msgs/msg/Float32           → data REAL
    rosgraph_msgs/msg/Clock        → clock_sec INTEGER, clock_nanosec INTEGER
    actuation_msgs/msg/Actuators   → normalized_json, angular_velocities_json
    (anything else)                → raw_bytes BLOB

Requirements:
    ros-<distro>-rosbag2-py   (ships with any ROS 2 desktop install)
    rclpy                     (for CDR deserialisation)
"""

from __future__ import annotations

import argparse
import json
import re
import sqlite3
import sys
from pathlib import Path
from typing import Any

# ── ROS imports — fail loudly so the user knows what's missing ────────────────
try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError as e:
    sys.exit(
        f"Import error: {e}\n"
        "Source your ROS 2 workspace first:\n"
        "    source /opt/ros/<distro>/setup.bash\n"
        "    source ~/ros2_ws/install/setup.bash   # if using a local workspace\n"
    )

# ── Helpers ───────────────────────────────────────────────────────────────────

def _topic_to_table(topic: str) -> str:
    """Turn '/bot_1/cmd_vel' into a valid SQLite table name like 'bot_1__cmd_vel'."""
    name = topic.lstrip("/").replace("/", "__")
    name = re.sub(r"[^a-zA-Z0-9_]", "_", name)
    if name and name[0].isdigit():
        name = "t_" + name
    return name or "unknown"


def _vec3(v) -> tuple[float, float, float]:
    return v.x, v.y, v.z


def _quat(q) -> tuple[float, float, float, float]:
    return q.x, q.y, q.z, q.w


# ── Per-type schema + row extractor ──────────────────────────────────────────

# Each entry: (columns_def: str, extractor: callable(msg) -> tuple)
# columns_def uses SQLite type affinity.

_SCHEMA: dict[str, tuple[str, Any]] = {}


def _reg(ros_type: str, cols: str, fn):
    _SCHEMA[ros_type] = (cols, fn)


_reg(
    "geometry_msgs/msg/Twist",
    "linear_x REAL, linear_y REAL, linear_z REAL, "
    "angular_x REAL, angular_y REAL, angular_z REAL",
    lambda m: (*_vec3(m.linear), *_vec3(m.angular)),
)

_reg(
    "nav_msgs/msg/Odometry",
    "pose_x REAL, pose_y REAL, pose_z REAL, "
    "pose_qx REAL, pose_qy REAL, pose_qz REAL, pose_qw REAL, "
    "vel_lin_x REAL, vel_lin_y REAL, vel_lin_z REAL, "
    "vel_ang_x REAL, vel_ang_y REAL, vel_ang_z REAL",
    lambda m: (
        *_vec3(m.pose.pose.position),
        *_quat(m.pose.pose.orientation),
        *_vec3(m.twist.twist.linear),
        *_vec3(m.twist.twist.angular),
    ),
)

_reg(
    "sensor_msgs/msg/Imu",
    "ori_x REAL, ori_y REAL, ori_z REAL, ori_w REAL, "
    "ang_vel_x REAL, ang_vel_y REAL, ang_vel_z REAL, "
    "lin_acc_x REAL, lin_acc_y REAL, lin_acc_z REAL",
    lambda m: (
        *_quat(m.orientation),
        *_vec3(m.angular_velocity),
        *_vec3(m.linear_acceleration),
    ),
)

_reg(
    "sensor_msgs/msg/JointState",
    "name_json TEXT, pos_json TEXT, vel_json TEXT, eff_json TEXT",
    lambda m: (
        json.dumps(list(m.name)),
        json.dumps(list(m.position)),
        json.dumps(list(m.velocity)),
        json.dumps(list(m.effort)),
    ),
)

_reg(
    "sensor_msgs/msg/Image",
    "width INTEGER, height INTEGER, encoding TEXT, step INTEGER, data BLOB",
    lambda m: (m.width, m.height, m.encoding, m.step, bytes(m.data)),
)

_reg(
    "sensor_msgs/msg/PointCloud2",
    "width INTEGER, height INTEGER, point_step INTEGER, row_step INTEGER, data BLOB",
    lambda m: (m.width, m.height, m.point_step, m.row_step, bytes(m.data)),
)

_reg(
    "std_msgs/msg/Bool",
    "data INTEGER",
    lambda m: (int(m.data),),
)

_reg(
    "std_msgs/msg/Float64",
    "data REAL",
    lambda m: (m.data,),
)

_reg(
    "std_msgs/msg/Float32",
    "data REAL",
    lambda m: (m.data,),
)

_reg(
    "rosgraph_msgs/msg/Clock",
    "clock_sec INTEGER, clock_nanosec INTEGER",
    lambda m: (m.clock.sec, m.clock.nanosec),
)

_reg(
    "actuation_msgs/msg/Actuators",
    "normalized_json TEXT, angular_velocities_json TEXT",
    lambda m: (
        json.dumps(list(m.normalized)),
        json.dumps(list(m.angular_velocities)),
    ),
)

_FALLBACK_COLS = "raw_bytes BLOB"
_FALLBACK_FN   = lambda m, raw: (raw,)   # noqa: E731


# ── Core conversion ───────────────────────────────────────────────────────────

def convert(bag_path: str | Path, db_path: str | Path) -> None:
    bag_path = Path(bag_path)
    db_path  = Path(db_path)

    if not bag_path.exists():
        sys.exit(f"Bag not found: {bag_path}")

    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_path),
        storage_id="sqlite3",   # default ROS 2 bag format
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # Build topic → type map
    topic_types: dict[str, str] = {
        ti.name: ti.type
        for ti in reader.get_all_topics_and_types()
    }

    print(f"Bag:    {bag_path}")
    print(f"Output: {db_path}")
    print(f"Topics ({len(topic_types)}):")
    for t, ty in sorted(topic_types.items()):
        print(f"  {t:<45} {ty}")
    print()

    # Pre-load message classes
    msg_classes: dict[str, type] = {}
    for topic, ros_type in topic_types.items():
        try:
            msg_classes[topic] = get_message(ros_type)
        except Exception as exc:
            print(f"  [warn] cannot load type {ros_type} for {topic}: {exc}")

    con = sqlite3.connect(str(db_path))
    cur = con.cursor()

    # Create tables
    tables_created: set[str] = set()

    def _ensure_table(table: str, extra_cols: str) -> None:
        if table in tables_created:
            return
        cur.execute(
            f"CREATE TABLE IF NOT EXISTS [{table}] ("
            f"  id INTEGER PRIMARY KEY,"
            f"  timestamp_ns INTEGER NOT NULL,"
            f"  timestamp_s  REAL    NOT NULL,"
            f"  {extra_cols}"
            f")"
        )
        cur.execute(
            f"CREATE INDEX IF NOT EXISTS idx_{table}_ts ON [{table}](timestamp_ns)"
        )
        tables_created.add(table)

    # Create a metadata table
    cur.execute(
        "CREATE TABLE IF NOT EXISTS _bag_metadata ("
        "  key TEXT PRIMARY KEY, value TEXT"
        ")"
    )
    cur.execute(
        "INSERT OR REPLACE INTO _bag_metadata VALUES ('bag_path', ?)",
        (str(bag_path),),
    )
    cur.execute(
        "INSERT OR REPLACE INTO _bag_metadata VALUES ('topics_json', ?)",
        (json.dumps(topic_types),),
    )

    # Stream messages
    counts: dict[str, int] = {}
    batch: dict[str, list] = {}
    BATCH_SIZE = 500

    def _flush(table: str) -> None:
        rows = batch.get(table)
        if not rows:
            return
        placeholders = ", ".join("?" * len(rows[0]))
        cur.executemany(
            f"INSERT INTO [{table}] VALUES ({placeholders})", rows
        )
        batch[table] = []

    total = 0
    while reader.has_next():
        topic, raw, recv_ns = reader.read_next()
        total += 1

        if total % 5000 == 0:
            print(f"  ... {total:,} messages processed", end="\r", flush=True)

        ros_type  = topic_types.get(topic, "")
        table     = _topic_to_table(topic)
        ts_s      = recv_ns * 1e-9

        if ros_type in _SCHEMA:
            cols_def, extractor = _SCHEMA[ros_type]
            _ensure_table(table, cols_def)
            try:
                msg = deserialize_message(raw, msg_classes[topic])
                row = (None, recv_ns, ts_s, *extractor(msg))
            except Exception:
                # Deserialisation failed — store raw
                _ensure_table(table + "_raw", _FALLBACK_COLS)
                row = (None, recv_ns, ts_s, raw)
                table = table + "_raw"
        else:
            _ensure_table(table, _FALLBACK_COLS)
            row = (None, recv_ns, ts_s, raw)

        batch.setdefault(table, []).append(row)
        counts[table] = counts.get(table, 0) + 1

        if len(batch[table]) >= BATCH_SIZE:
            _flush(table)

    # Flush remainder
    for table in list(batch):
        _flush(table)

    con.commit()
    con.close()

    print(f"\nDone — {total:,} messages written to {db_path}")
    print("\nRow counts per table:")
    for table, n in sorted(counts.items()):
        print(f"  {table:<50} {n:>8,}")


# ── CLI ───────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert a ROS 2 bag to SQLite for analysis.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("bag_path", help="Path to the ROS 2 bag directory or .db3 file")
    parser.add_argument(
        "output",
        nargs="?",
        help="Output SQLite file (default: <bag_name>.db)",
    )
    parser.add_argument(
        "--storage",
        default="sqlite3",
        help="rosbag2 storage plugin (default: sqlite3; use 'mcap' for .mcap bags)",
    )
    args = parser.parse_args()

    bag_path = Path(args.bag_path)
    if args.output:
        db_path = Path(args.output)
    else:
        db_path = bag_path.with_suffix("").with_suffix(".db") \
            if bag_path.is_file() \
            else bag_path.parent / (bag_path.name + ".db")

    convert(bag_path, db_path)


if __name__ == "__main__":
    main()

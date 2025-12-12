#!/usr/bin/env python3
"""Skeleton: inspect the bundled ROS2 SQLite bag (student fills in the logic)."""

import sqlite3
from pathlib import Path

# Optional if you want to deserialize messages instead of printing raw bytes:
# from rclpy.serialization import deserialize_message
# from sensor_msgs.msg import JointState


def main():
    bag_path = Path(__file__).resolve().parent / 'bag' / 'data_ros2' / 'data_ros2.db3'
    if not bag_path.exists():
        print(f"Bag not found: {bag_path}")
        return

    # TODO: open sqlite3 connection, list topics (table `topics`), count messages
    # in `messages` per topic_id, and optionally decode the first JointState.
    # Example queries:
    #   SELECT id, name, type FROM topics;
    #   SELECT COUNT(*) FROM messages WHERE topic_id=?;
    #   SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp LIMIT 1;
    raise NotImplementedError("Fill in the bag inspection logic for Lab 10")


if __name__ == '__main__':
    main()

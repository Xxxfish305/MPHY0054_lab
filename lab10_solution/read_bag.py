#!/usr/bin/env python3
"""Lab 10 solution: inspect ROS2 SQLite bag without ROS launch."""
import sqlite3
from pathlib import Path
from typing import List, Tuple

from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState


def list_topics(conn: sqlite3.Connection) -> List[Tuple[int, str, str]]:
    cur = conn.cursor()
    cur.execute("SELECT id, name, type FROM topics")
    return cur.fetchall()


def count_messages(conn: sqlite3.Connection, topic_id: int) -> int:
    cur = conn.cursor()
    cur.execute("SELECT COUNT(*) FROM messages WHERE topic_id = ?", (topic_id,))
    row = cur.fetchone()
    return row[0] if row else 0


def read_first_message(conn: sqlite3.Connection, topic_id: int):
    cur = conn.cursor()
    cur.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp LIMIT 1",
        (topic_id,),
    )
    return cur.fetchone()


def main():
    bag_path = Path(__file__).resolve().parent / 'bag' / 'data_ros2' / 'data_ros2.db3'
    if not bag_path.exists():
        print(f"Bag not found: {bag_path}")
        return

    print(f"Using bag: {bag_path}")
    conn = sqlite3.connect(bag_path)
    try:
        topics = list_topics(conn)
        print("Topics:")
        for tid, name, mtype in topics:
            print(f"  id={tid} name={name} type={mtype} count={count_messages(conn, tid)}")

        js_topics = [t for t in topics if 'JointState' in t[2]]
        if js_topics:
            tid, name, _ = js_topics[0]
            row = read_first_message(conn, tid)
            if row:
                ts, raw = row
                msg = deserialize_message(raw, JointState)
                print(f"\nFirst JointState on '{name}' (timestamp={ts}):")
                print(f"  names: {list(msg.name)}")
                print(f"  positions: {list(msg.position)}")
            else:
                print("No JointState messages found.")
        else:
            print("No JointState topic present.")
    finally:
        conn.close()


if __name__ == '__main__':
    main()

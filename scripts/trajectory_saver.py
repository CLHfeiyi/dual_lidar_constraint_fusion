#!/usr/bin/env python3
"""
Save LIO-SAM odometry to TUM-format trajectory file directly.
More reliable than rosbag record (no file corruption risk).

Usage in launch file:
  <node pkg="lio_sam" type="trajectory_saver.py" name="trajectory_saver" output="screen">
      <param name="output_path" value="/path/to/trajectory.tum" />
  </node>
"""

import os
import signal
import sys

import rospy
from nav_msgs.msg import Odometry


class TrajectorySaver:
    def __init__(self):
        output = rospy.get_param("~output_path", "/tmp/lio_sam_traj.tum")
        self.topic = rospy.get_param("~odom_topic", "/lio_sam/mapping/odometry")

        os.makedirs(os.path.dirname(output) or ".", exist_ok=True)
        self.f = open(output, "w")
        self.count = 0

        self.sub = rospy.Subscriber(
            self.topic, Odometry, self.callback, queue_size=1000)

        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo("TrajectorySaver: %s -> %s", self.topic, output)

    def callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t = msg.header.stamp.to_sec()
        self.f.write(f"{t:.9f} {p.x:.6f} {p.y:.6f} {p.z:.6f} "
                     f"{q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n")
        self.count += 1
        if self.count % 500 == 0:
            self.f.flush()

    def on_shutdown(self):
        self.f.flush()
        self.f.close()
        rospy.loginfo("TrajectorySaver: saved %d poses", self.count)


def main():
    rospy.init_node("trajectory_saver", disable_signals=False)
    TrajectorySaver()
    rospy.spin()


if __name__ == "__main__":
    main()

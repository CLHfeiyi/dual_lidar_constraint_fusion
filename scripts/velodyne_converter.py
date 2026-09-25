#!/usr/bin/env python3
"""ROS node that converts VelodyneScan packets to PointCloud2 in real-time.

Usage: rosrun <pkg> velodyne_scan_to_cloud.py
  Subscribes: /velodyne_packets (velodyne_msgs/VelodyneScan)
  Publishes:  /velodyne_points (sensor_msgs/PointCloud2)
"""

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2

try:
    import velodyne_decoder as vd
    HAS_VD = True
except ImportError:
    HAS_VD = False


class VelodyneConverter:
    def __init__(self):
        rospy.init_node('velodyne_converter')
        self.pub = rospy.Publisher('/velodyne_points', PointCloud2, queue_size=10)

        if HAS_VD:
            rospy.loginfo("Using velodyne_decoder for packet decoding")
            self.decoder = vd.ScanDecoder(vd.Config(model=vd.Model.VLP16,
                                                     min_range=0.5, max_range=100.0))
            self.use_builtin = False
        else:
            rospy.logwarn("velodyne_decoder not available, using manual decode")
            self.use_builtin = True

        # Need velodyne_msgs for the message type
        try:
            from velodyne_msgs.msg import VelodyneScan
            self.VelodyneScan = VelodyneScan
        except ImportError:
            rospy.logerr("velodyne_msgs not available. Please install ros-noetic-velodyne-msgs")
            raise

        self.sub = rospy.Subscriber('/velodyne_packets', self.VelodyneScan, self.callback)
        rospy.loginfo("Velodyne converter ready")

    def decode_vlp16_packet(self, data):
        """Manual VLP-16 packet decode returning Nx4 [x,y,z,intensity]."""
        VERT_ANGLES = np.array(
            [-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15],
            dtype=np.float64) * np.pi / 180.0
        DIST_RES = 0.002
        all_pts = []

        for block_idx in range(12):
            off = 42 + block_idx * 100
            flag = int.from_bytes(data[off:off+2], 'little')
            if flag != 0xEEFF:
                continue
            azimuth = int.from_bytes(data[off+2:off+4], 'little') / 100.0
            az_rad = np.deg2rad(azimuth)

            meas_off = off + 4
            for si in range(32):
                li = si % 16
                di = meas_off + si * 3
                dist = int.from_bytes(data[di:di+2], 'little') * DIST_RES
                if dist < 0.5 or dist > 100.0:
                    continue
                intensity = data[di + 2]
                v_ang = VERT_ANGLES[li]
                az = az_rad + (si / 32.0) * (0.2 * np.pi / 180.0)
                cos_v = np.cos(v_ang)
                all_pts.append([
                    dist * cos_v * np.cos(az),
                    dist * cos_v * np.sin(az),
                    dist * np.sin(v_ang),
                    float(intensity)
                ])
        return np.array(all_pts, dtype=np.float32) if all_pts else np.zeros((0, 4), dtype=np.float32)

    def make_cloud(self, points, stamp):
        if points.shape[0] == 0:
            return None
        hdr = Header(stamp=stamp, frame_id="velodyne")
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('intensity', 12, PointField.FLOAT32, 1),
        ]
        return pc2.create_cloud(hdr, fields, points)

    def callback(self, msg):
        try:
            if self.use_builtin:
                all_pts = []
                for pkt in msg.packets:
                    pts = self.decode_vlp16_packet(pkt.data)
                    if len(pts) > 0:
                        all_pts.append(pts)
                points = np.vstack(all_pts) if all_pts else np.zeros((0, 4), dtype=np.float32)
            else:
                _, points_all = self.decoder.decode_message(msg)
                points = points_all[:, :4]

            cloud = self.make_cloud(points, msg.header.stamp)
            if cloud is not None:
                self.pub.publish(cloud)
        except Exception as e:
            rospy.logerr_throttle(10, f"Conversion error: {e}")

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    VelodyneConverter().run()

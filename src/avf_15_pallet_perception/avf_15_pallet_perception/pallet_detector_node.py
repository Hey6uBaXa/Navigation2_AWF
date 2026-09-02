#!/usr/bin/env python3
"""
Markerless pallet detector.

Idea:
  - Take the RGBD point cloud from the front depth camera.
  - Transform it into base_link.
  - Keep only the "fork band" slice (min_height..max_height) where the
    pallet's two openings are expected, within a forward ROI.
  - Project to 2D (x forward, y lateral) and cluster remaining points
    with DBSCAN -> each cluster is a solid part of the pallet
    (front board / legs). The pallet has a fixed known width and two
    openings between three legs: left leg - opening - center leg -
    opening - right leg. We look for two clusters separated by a gap
    close to the expected fork spacing, in front of the robot.
  - Publish a PoseStamped for the *pallet insertion point* (mid-point
    between the two openings, oriented to face the robot / dock),
    at a steady rate, on a topic that docking_server consumes as
    external_detection_pose.

This is a heuristic, not a full segmentation pipeline: tune
PALLET_OPENING_WIDTH / PALLET_LEG_WIDTH / band heights for your
actual pallet model and camera mount.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import tf2_ros
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped

try:
    from sklearn.cluster import DBSCAN
    HAVE_SKLEARN = True
except ImportError:
    HAVE_SKLEARN = False


class PalletDetectorNode(Node):

    def __init__(self):
        super().__init__('pallet_detector_node')

        # ---- Параметры (настройте под свою паллету / крепление камеры) ----
        self.declare_parameter('cloud_topic', 'rear_depth_camera/points')  # топик облака точек
        self.declare_parameter('base_frame', 'robot01/base_link'
)                  # система координат базы робота
        self.declare_parameter('pose_topic', 'detected_dock_pose')   # топик публикации позы стыковки
        self.declare_parameter('found_topic', 'pallet_detector/pallet_found')  # топик флага "паллета найдена"

        # Область интереса (ROI) в системе координат base_link
        # (вперёд = +x согласно конвенции SDF, проверьте свой TF)
        self.declare_parameter('roi_x_min', -10.0)         # ближняя граница ROI по x, м
        self.declare_parameter('roi_x_max', 10.0)         # дальняя граница ROI по x, м
        self.declare_parameter('roi_y_abs_max', 1.2)     # макс. |y| — ширина ROI влево/вправо, м

        # Диапазон высот для проёмов под вилы (высота над base_link, где находятся отверстия паллеты)
        self.declare_parameter('band_min_height', 0.04)  # нижняя граница полосы поиска отверстий, м
        self.declare_parameter('band_max_height', 0.07)  # верхняя граница полосы поиска отверстий, м

        # Геометрия паллеты (примерные значения для EUR-паллеты) — ТРЕБУЕТ НАСТРОЙКИ
        self.declare_parameter('opening_width', 0.227)       # ширина проёма для вил, м
        self.declare_parameter('opening_width_tol', 0.04)   # допуск по ширине проёма, м
        self.declare_parameter('leg_spacing', 0.382)         # расстояние от центральной ножки до крайней, м
        self.declare_parameter('leg_spacing_tol', 0.08)     # допуск по расстоянию между ножками, м

        # Параметры кластеризации DBSCAN
        self.declare_parameter('dbscan_eps', 0.008)          # радиус окрестности (eps), м
        self.declare_parameter('dbscan_min_samples', 4)     # мин. число точек для формирования кластера

        self.declare_parameter('publish_rate_hz', 5.0)      # частота публикации результата, Гц

        self.cloud_topic = self.get_parameter('cloud_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.found_topic = self.get_parameter('found_topic').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos = QoSProfile(depth=1,
                          reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST)

        self.cloud_sub = self.create_subscription(
            PointCloud2, self.cloud_topic, self.cloud_cb, qos)

        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self.found_pub = self.create_publisher(Bool, self.found_topic, 10)

        self.last_cloud_msg = None
        rate = self.get_parameter('publish_rate_hz').value
        self.timer = self.create_timer(1.0 / rate, self.process_and_publish)

        if not HAVE_SKLEARN:
            self.get_logger().warn(
                'scikit-learn not available, falling back to a simple '
                '1D-histogram clustering (less robust). '
                'Install python3-sklearn for DBSCAN.')

        self.get_logger().info(
            f'pallet_detector_node started, listening on {self.cloud_topic}')

    # ------------------------------------------------------------------
    def cloud_cb(self, msg: PointCloud2):
        self.last_cloud_msg = msg

    # ------------------------------------------------------------------
    def process_and_publish(self):
        if self.last_cloud_msg is None:
            return
        msg = self.last_cloud_msg

        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, msg.header.frame_id, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(f'TF lookup failed: {ex}', throttle_duration_sec=2.0)
            return

        pts = self._cloud_to_base_frame(msg, tf)

        # self.get_logger().info(
        #     f'x: {np.nanmin(pts[:,0]):.2f} .. {np.nanmax(pts[:,0]):.2f}'
        # )
        # self.get_logger().info(
        #     f'y: {np.nanmin(pts[:,1]):.2f} .. {np.nanmax(pts[:,1]):.2f}'
        # )
        # self.get_logger().info(
        #     f'z: {np.nanmin(pts[:,2]):.2f} .. {np.nanmax(pts[:,2]):.2f}'
        # )
        

        n_total = 0 if pts is None else len(pts)
        if pts is None or len(pts) < 20:
            self.get_logger().info(
                f'DEBUG: total transformed points = {n_total} (<20, aborting)',
                throttle_duration_sec=1.0)
            self._publish_found(False)
            return

        #floor_removed = pts[pts[:,2] > 0.02]
        floor_removed = pts[
            (pts[:,2] > 0.02) &
            (pts[:,2] < 0.03)
        ]
        # self.get_logger().info(
        #     f'after floor removal: {floor_removed.shape[0]}'
        # )

        band = self._filter_roi_band(floor_removed)

        # self.get_logger().info(
        #     f'after ROI: {band.shape[0]}'
        # )
        


        band = self._filter_roi_band(pts)
        self._publish_debug_cloud(band, msg.header.stamp)
        if band.shape[0] < 15:
            #self.get_logger().info(
            #    f'DEBUG: total={n_total} band(ROI+height)={band.shape[0]} (<15, aborting). '
            #   f'Check roi_x_min/max, roi_y_abs_max, band_min/max_height params.',
            #   throttle_duration_sec=1.0)
            self._publish_found(False)
            return

        n_clusters, n_candidates, pose_xy_yaw = self._find_pallet_opening_debug(band)
        if pose_xy_yaw is None:
            self.get_logger().warn(
                f'DEBUG: total={n_total} band={band.shape[0]} clusters={n_clusters} '
                f'candidates={n_candidates} (no matching opening gap found). '
                f'Check opening_width/opening_width_tol/dbscan_eps params.',
                throttle_duration_sec=1.0)
            self._publish_found(False)
            return

        x, y, yaw = pose_xy_yaw
        self.get_logger().info(
            f'DEBUG: pallet found at x={x:.2f} y={y:.2f} yaw={yaw:.2f} '
            f'(band={band.shape[0]} clusters={n_clusters})',
            throttle_duration_sec=1.0)
        self._publish_pose(x, y, yaw, msg.header.stamp)
        self._publish_found(True)

    # ------------------------------------------------------------------
    def _publish_debug_cloud(self, band: np.ndarray, stamp):
        """Publish the filtered ROI+height-band cloud so it can be viewed
        in RViz (add a PointCloud2 display on /pallet_detector/debug_band_cloud,
        Fixed Frame = base_link) to visually check ROI/height parameters
        against where the pallet actually is."""
        if not hasattr(self, 'debug_cloud_pub'):
            self.debug_cloud_pub = self.create_publisher(
                PointCloud2, 'pallet_detector/debug_band_cloud', 5)
        header = self.get_clock().now().to_msg()
        msg = pc2.create_cloud_xyz32(
            self._make_header(stamp), band.astype(np.float32).tolist()
            if band.shape[0] < 20000 else band[:20000].astype(np.float32).tolist())
        self.debug_cloud_pub.publish(msg)

    def _make_header(self, stamp):
        from std_msgs.msg import Header
        h = Header()
        h.frame_id = self.base_frame
        h.stamp = stamp
        return h

    # ------------------------------------------------------------------
    def _cloud_to_base_frame(self, msg: PointCloud2, tf):
        """Read xyz points and bulk-transform into base_frame."""
        arr = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if arr is None or arr.size == 0:
            return None

        # read_points returns either a structured array (fields x,y,z) or
        # already a plain (N,3) array depending on sensor_msgs_py version.
        if arr.dtype.names is not None:
            arr = np.stack(
                [arr['x'].astype(np.float32),
                 arr['y'].astype(np.float32),
                 arr['z'].astype(np.float32)], axis=-1)
        else:
            arr = np.asarray(arr, dtype=np.float32)

        arr = arr.reshape(-1, 3)
        if arr.shape[0] == 0:
            return None

        # Build 4x4 transform matrix from the TransformStamped once,
        # then apply to all points (much faster than per-point tf2 calls)
        t = tf.transform.translation
        q = tf.transform.rotation
        T = self._quat_trans_to_matrix(q, t)

        # homog = np.hstack([arr, np.ones((arr.shape[0], 1), dtype=np.float32)])
        # out = (T @ homog.T).T[:, :3]
        # return out
        homog = np.hstack([
            arr,
            np.ones((arr.shape[0], 1), dtype=np.float32)
        ])

        out = (T @ homog.T).T[:, :3]

        # удалить NaN и Inf
        out = out[np.isfinite(out).all(axis=1)]

        if out.shape[0] == 0:
            return None

        return out

    @staticmethod
    def _quat_trans_to_matrix(q, t):
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2*(y*y+z*z), 2*(x*y-z*w),     2*(x*z+y*w)],
            [2*(x*y+z*w),     1 - 2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w),     2*(y*z+x*w),     1 - 2*(x*x+y*y)],
        ])
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [t.x, t.y, t.z]
        return T

    # ------------------------------------------------------------------
    def _filter_roi_band(self, pts: np.ndarray) -> np.ndarray:
        x_min = self.get_parameter('roi_x_min').value
        x_max = self.get_parameter('roi_x_max').value
        y_abs = self.get_parameter('roi_y_abs_max').value
        z_min = self.get_parameter('band_min_height').value
        z_max = self.get_parameter('band_max_height').value

        mask = (
            (pts[:, 0] > x_min) & (pts[:, 0] < x_max) &
            (np.abs(pts[:, 1]) < y_abs) &
            (pts[:, 2] > z_min) & (pts[:, 2] < z_max)
        )
        return pts[mask]

    # ------------------------------------------------------------------
    def _find_pallet_opening_debug(self, band: np.ndarray):
        """Same as _find_pallet_opening but also returns cluster/candidate
        counts for diagnostics."""
        xy = band[:, :2]
        labels = self._cluster(xy)
        if labels is None:
            return 0, 0, None
        unique = [l for l in set(labels) if l != -1]
        if len(unique) < 2:
            return len(unique), 0, None

        clusters = []
        for l in unique:
            c = xy[labels == l]
            clusters.append({
                'y_mean': float(np.mean(c[:, 1])),
                'x_mean': float(np.mean(c[:, 0])),
                'y_min': float(np.min(c[:, 1])),
                'y_max': float(np.max(c[:, 1])),
                'n': c.shape[0],
            })
        clusters.sort(key=lambda c: c['y_mean'])

        self.get_logger().info(
            f'clusters found: {len(unique)}'
        )
        for l in unique:
            c = xy[labels == l]

            self.get_logger().info(
                f'cluster size={c.shape[0]} '
                f'x=[{c[:,0].min():.2f},{c[:,0].max():.2f}] '
                f'y=[{c[:,1].min():.2f},{c[:,1].max():.2f}]'
            )

        opening_w = self.get_parameter('opening_width').value
        opening_tol = self.get_parameter('opening_width_tol').value

        candidates = []
        for i in range(len(clusters) - 1):
            gap = clusters[i + 1]['y_min'] - clusters[i]['y_max']
            if abs(gap - opening_w) < opening_tol:
                mid_y = 0.5 * (clusters[i + 1]['y_min'] + clusters[i]['y_max'])
                mid_x = 0.5 * (clusters[i]['x_mean'] + clusters[i + 1]['x_mean'])
                candidates.append((mid_x, mid_y))

        if not candidates:
            # log actual gaps found, to help re-tune opening_width
            gaps = [round(clusters[i + 1]['y_min'] - clusters[i]['y_max'], 3)
                    for i in range(len(clusters) - 1)]
            self.get_logger().warn(
                f'DEBUG: cluster gaps found (m): {gaps}, expected opening_width='
                f'{opening_w}+-{opening_tol}', throttle_duration_sec=1.0)
            return len(unique), 0, None

        cx = float(np.mean([c[0] for c in candidates]))
        cy = float(np.mean([c[1] for c in candidates]))
        return len(unique), len(candidates), (cx, cy, 0.0)

    # ------------------------------------------------------------------
    def _find_pallet_opening(self, band: np.ndarray):
        """
        band: Nx3 points (base_link frame) inside the fork-insertion height
        band. Cluster in the x-y ground plane; solid pallet material
        (legs/front board) shows up as clusters, the two fork slots show
        up as gaps between clusters at a consistent lateral spacing.
        Returns (x, y, yaw) of the pallet insertion centre, or None.
        """
        xy = band[:, :2]

        labels = self._cluster(xy)
        if labels is None:
            return None

        unique = [l for l in set(labels) if l != -1]
        if len(unique) < 2:
            return None

        # Cluster centroids and lateral extents
        clusters = []
        for l in unique:
            c = xy[labels == l]
            clusters.append({
                'y_mean': float(np.mean(c[:, 1])),
                'x_mean': float(np.mean(c[:, 0])),
                'y_min': float(np.min(c[:, 1])),
                'y_max': float(np.max(c[:, 1])),
                'n': c.shape[0],
            })
        clusters.sort(key=lambda c: c['y_mean'])

        opening_w = self.get_parameter('opening_width').value
        opening_tol = self.get_parameter('opening_width_tol').value

        # scan adjacent cluster pairs for a gap matching the fork opening
        candidates = []
        for i in range(len(clusters) - 1):
            gap = clusters[i + 1]['y_min'] - clusters[i]['y_max']
            if abs(gap - opening_w) < opening_tol:
                mid_y = 0.5 * (clusters[i + 1]['y_min'] + clusters[i]['y_max'])
                mid_x = 0.5 * (clusters[i]['x_mean'] + clusters[i + 1]['x_mean'])
                candidates.append((mid_x, mid_y))

        if not candidates:
            return None

        # if we found the left and right slot, the pallet centre is the
        # mean of both slot midpoints; if only one slot, use it directly
        cx = float(np.mean([c[0] for c in candidates]))
        cy = float(np.mean([c[1] for c in candidates]))

        # Orientation: face the pallet straight-on (front board assumed
        # parallel to the y axis at this stage of the pipeline). Refine
        # yaw from the leading edge of the front-most cluster if needed.
        yaw = 0.0
        return cx, cy, yaw

    # ------------------------------------------------------------------
    def _cluster(self, xy: np.ndarray):
        eps = self.get_parameter('dbscan_eps').value
        min_samples = self.get_parameter('dbscan_min_samples').value

        if HAVE_SKLEARN:
            db = DBSCAN(eps=eps, min_samples=min_samples).fit(xy)
            return db.labels_

        # Fallback: crude 1D clustering on y using a sorted-gap split
        order = np.argsort(xy[:, 1])
        y_sorted = xy[order, 1]
        labels = np.zeros(len(y_sorted), dtype=int)
        label = 0
        for i in range(1, len(y_sorted)):
            if y_sorted[i] - y_sorted[i - 1] > eps * 3:
                label += 1
            labels[i] = label
        out = np.empty(len(xy), dtype=int)
        out[order] = labels
        return out

    # ------------------------------------------------------------------
    def _publish_pose(self, x, y, yaw, stamp):
        msg = PoseStamped()
        msg.header.frame_id = self.base_frame
        msg.header.stamp = stamp
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        msg.pose.orientation.z = float(np.sin(yaw / 2.0))
        msg.pose.orientation.w = float(np.cos(yaw / 2.0))
        self.pose_pub.publish(msg)

    def _publish_found(self, found: bool):
        m = Bool()
        m.data = found
        self.found_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = PalletDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

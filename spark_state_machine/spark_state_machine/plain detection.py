import pyrealsense2 as rs
import numpy as np
import open3d as o3d
import cv2
from scipy.spatial.transform import Rotation as R

def estimate_orientation_from_plane():
    # ==== RealSense Init ====
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    profile = pipeline.start(config)
    
    align = rs.align(rs.stream.color)
    intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
    intr = o3d.camera.PinholeCameraIntrinsic(
        width=intrinsics.width,
        height=intrinsics.height,
        fx=intrinsics.fx,
        fy=intrinsics.fy,
        cx=intrinsics.ppx,
        cy=intrinsics.ppy
    )

    try:
        print("Capturing frame for plane detection...")
        for _ in range(10):  # Warm up camera
            pipeline.wait_for_frames()
        frames = pipeline.wait_for_frames()
        aligned = align.process(frames)

        depth_frame = aligned.get_depth_frame()
        if not depth_frame:
            print("Failed to get depth frame")
            return

        depth_image = np.asanyarray(depth_frame.get_data())
        depth_o3d = o3d.geometry.Image(depth_image)
        
        # === Create Point Cloud ===
        pcd = o3d.geometry.PointCloud.create_from_depth_image(
            depth_o3d, intr, depth_scale=1.0/1000.0, depth_trunc=2.0, stride=2)

        pcd = pcd.voxel_down_sample(voxel_size=0.005)
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

        # Check if enough points for plane segmentation
        if np.asarray(pcd.points).shape[0] < 3:
            print("Not enough points in point cloud for plane segmentation.")
            return None, None, None

        # === Plane Segmentation ===
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                                 ransac_n=3,
                                                 num_iterations=1000)
        [a, b, c, d] = plane_model
        normal = np.array([a, b, c])
        normal = normal / np.linalg.norm(normal)
        print(f"Plane normal: {normal}")

        # === Calculate Roll and Pitch ===
        # Camera looks along Z, so we compare normal to Z axis
        z_axis = np.array([0, 0, 1])
        rotation_vector = np.cross(normal, z_axis)
        angle = np.arccos(np.dot(normal, z_axis))
        
        if np.linalg.norm(rotation_vector) < 1e-6:
            r = R.identity()
        else:
            r = R.from_rotvec(rotation_vector / np.linalg.norm(rotation_vector) * angle)
        
        roll, pitch, yaw = r.as_euler('xyz', degrees=True)
        print(f"Estimated orientation from plane:\nRoll: {roll:.2f}°, Pitch: {pitch:.2f}°, Yaw: {yaw:.2f}°")

        # Optionally visualize
        plane = pcd.select_by_index(inliers)
        plane.paint_uniform_color([1.0, 0.5, 0])
        o3d.visualization.draw_geometries([plane], window_name="Detected Plane")

        return roll, pitch, yaw

    finally:
        pipeline.stop()

if __name__ == "__main__":
    roll, pitch, yaw = estimate_orientation_from_plane()

import json
import omni.replicator.core as rep
from PIL import Image, ImageDraw
import omni.syntheticdata as sd
import numpy as np
from omni.physx import get_physx_scene_query_interface
from isaacsim.sensors.camera import Camera
import os
import copy
import time
# Ensure replicator settings for RTSubframes is configured
rep.settings.carb_settings("/omni/replicator/RTSubframes", 20)

def world_to_image_pinhole(world_points, camera_params):
    """
    Projects 3D world points into 2D image points using pinhole camera model.
    """
    proj_mat = camera_params["cameraProjection"].reshape(4, 4)
    view_mat = camera_params["cameraViewTransform"].reshape(4, 4)
    view_proj_mat = np.dot(view_mat, proj_mat)
    world_points_homo = np.pad(world_points, ((0, 0), (0, 1)), constant_values=1.0)
    tf_points = np.dot(world_points_homo, view_proj_mat)
    tf_points = tf_points / tf_points[..., -1:]
    return 0.5 * (tf_points[..., :2] + 1)

def draw_points(img, image_points, labels=None, color=(255, 0, 0), radius=5):
    """
    Draws points and optional labels on a PIL image.
    """
    width, height = img.size
    draw = ImageDraw.Draw(img)
    image_points[:, 1] = height - image_points[:, 1]  # Flip Y-axis
    
    for i, point in enumerate(image_points):
        xy = (point[0] - radius, point[1] - radius, point[0] + radius, point[1] + radius)
        draw.ellipse(xy, fill=color, outline=(0, 0, 0))
        if labels:
            draw.text((point[0] + radius, point[1] + radius), str(labels[i]), fill=color)

def estimate_transform_3d(source_points, target_points):
    """
    Estimates the rigid body transform (rotation and translation)
    between two sets of 3D points using SVD.
    
    Args:
        source_points (np.ndarray): An [N, 3] array of source points.
        target_points (np.ndarray): An [N, 3] array of target points.

    Returns:
        tuple: A tuple containing the rotation matrix (3x3) and
               the translation vector (3,).
    """
    source_points = np.asarray(source_points, dtype=np.float32)
    target_points = np.asarray(target_points, dtype=np.float32)

    source_centroid = np.mean(source_points, axis=0)
    target_centroid = np.mean(target_points, axis=0)

    centered_source = source_points - source_centroid
    centered_target = target_points - target_centroid

    H = centered_source.T @ centered_target
    U, S, Vt = np.linalg.svd(H)

    R_candidate = Vt.T @ U.T
    if np.linalg.det(R_candidate) < 0:
        V = Vt.T
        V[:, -1] *= -1
        R = V @ U.T
    else:
        R = R_candidate

    t = target_centroid - R @ source_centroid
    return R, t

class Semantic3dPointPoseEstimationWriter(rep.Writer):
    def __init__(self, output_dir: str, target_prim: str = "/Replicator/Ref_Xform/Ref", semantic_json_path: str = "/home/user/isaac_save/3d_model_semantic/forklift_c.json", save_debug_interval: int = 1):
        """
        Initializes the writer for semantic 3D point pose estimation.
        """
        self.frame_id = 0
        self.backend = rep.BackendDispatch({"paths": {"out_dir": output_dir}})
        self.annotators = ["bounding_box_3d", "rgb", "camera_params"]
        self.raycast_distance_threshold = 0.3
        self.target_prim = target_prim
        self.debug_output_dir = os.path.join(output_dir, "debug")
        os.makedirs(self.debug_output_dir, exist_ok=True)
        with open(semantic_json_path, "r") as f:
            self.semantic_data = json.load(f)
        self.save_debug_interval = save_debug_interval

    def write(self, data):
        """
        The main method to process and write data for each frame.
        """
        print("--- Start of write method ---")
        print(f"Current frame_id: {self.frame_id}")
        
        camera_prim_path = "/World/Cameras/Camera"
        self.camera = Camera(camera_prim_path, resolution=(1280, 720))
        
        img = Image.fromarray(data["rgb"])
        img_original = copy.deepcopy(img)
        self.backend.write_image(f"{self.frame_id}.png", img_original)
        
        render_product = [k for k in data.keys() if k.startswith("rp_")][0]
        width, height = data[render_product]["resolution"]

        bbox_info = data["bounding_box_3d"]["info"]
        prim_paths = bbox_info["primPaths"]
        
        try:
            target_index = prim_paths.index(self.target_prim)
        except ValueError:
            print(f"[Warning] Target prim '{self.target_prim}' not found! Skipping frame {self.frame_id}")
            print("--- End of write method (skipped) ---")
            return

        bbox3ds = data["bounding_box_3d"]["data"]
        corners_3d_all = sd.helpers.get_bbox_3d_corners(bbox3ds)
        corners_3d = corners_3d_all[target_index]
        
        # Get the shifted bbox corners from the JSON data
        bbox_corners_shifted_local = np.array(self.semantic_data["bbox_corners_shifted"])

        # Estimate R, t from the local shifted bbox points to the world-space corners
        R_est, t_est = estimate_transform_3d(bbox_corners_shifted_local, corners_3d)
        print("Estimated Rotation Matrix (R):\n", np.round(R_est, 4))
        print("Estimated Translation Vector (t):\n", np.round(t_est, 4))
        
        # Denormalize the semantic points
        sem_norm = np.array([p["location"] for p in self.semantic_data["semantic_points"]])
        sem_name = np.array([p["name"] for p in self.semantic_data["semantic_points"]])
        
        # The shifted bbox corner `0` is at the origin, and `7` represents the size.
        bbl_shifted = bbox_corners_shifted_local[0]
        ftr_shifted = bbox_corners_shifted_local[7]
        bbox_size_local = ftr_shifted - bbl_shifted
        
        # Denormalize the semantic points from [0,1] space to local space
        sem_local = sem_norm * bbox_size_local + bbl_shifted
        print("Denormalized semantic points (local):", np.round(sem_local, 4))
        
        # Transform the denormalized local points to world coordinates
        sem_world = (R_est @ sem_local.T).T + t_est
        print("Transformed semantic points (world):", np.round(sem_world, 4))

        sem_2d = world_to_image_pinhole(sem_world, data["camera_params"])
        sem_2d_pixels = sem_2d * np.array([[width, height]])
        
        corners_2d = world_to_image_pinhole(corners_3d, data["camera_params"])
        corners_2d_pixels = corners_2d * np.array([[width, height]])

        scene_query = get_physx_scene_query_interface()
        camera_origin, _ = self.camera.get_world_pose()
        camera_origin = np.array(camera_origin)
        
        all_semantic_points = []
        hit_points_2d = []
        hit_labels = []
        

        for i, point_2d_pixels in enumerate(sem_2d_pixels):
            print(f"\n--- Raycast check for semantic point {i} ({sem_name[i]}) ---")
            print(f"    World position: {np.round(sem_world[i], 4)}")
            print(f"    Projected pixel (before occlusion check): {np.round(point_2d_pixels, 2)}")

            is_occluded = False
            ray_dir = sem_world[i] - camera_origin
            ray_dir_normalized = ray_dir / np.linalg.norm(ray_dir)
            dist_to_sem_point = np.linalg.norm(sem_world[i] - camera_origin)

            print(f"    Camera origin: {np.round(camera_origin, 4)}")
            print(f"    Ray direction (normalized): {np.round(ray_dir_normalized, 6)}")
            print(f"    Distance to semantic point: {dist_to_sem_point:.4f}")

            for _ in range(1):
                hit = scene_query.raycast_closest(
                    origin=camera_origin,
                    dir=ray_dir_normalized,
                    distance=1000.0
                )

            if hit:
                hit_pos_3d = hit['position']
                hit_distance = hit['distance']
                print(f"    [Hit] Position: {np.round(hit_pos_3d, 4)}, Distance: {hit_distance:.4f}")

                hit_pos_2d = world_to_image_pinhole(np.array([hit_pos_3d]), data["camera_params"])
                hit_pos_2d *= np.array([[width, height]])
                print(f"    [Hit] Projected to pixel: {np.round(hit_pos_2d[0], 2)}")

                hit_points_2d.append(hit_pos_2d[0])
                hit_labels.append(sem_name[i])

                if hit_distance < dist_to_sem_point - self.raycast_distance_threshold:
                    is_occluded = True
                    print(f"    --> Marked as OCCLUDED (hit distance {hit_distance:.4f} < semantic distance {dist_to_sem_point:.4f})")
                else:
                    print(f"    --> Marked as VISIBLE (semantic point is first hit)")

            else:
                print("    [No Hit] Raycast did not hit any object.")
                print("    --> Marked as VISIBLE (no occluder found)")

            all_semantic_points.append({
                "point_2d_pixels": point_2d_pixels,
                "label": sem_name[i],
                "is_occluded": is_occluded
            })
            print(f"    Final occlusion status for {sem_name[i]}: {is_occluded}")
            print("--- End of raycast check ---\n")

        # Save debug images for the first 10 frames and then every `self.save_debug_interval` frames
        if self.frame_id < 10 or self.frame_id % self.save_debug_interval == 0:
            visible_points_2d = [p["point_2d_pixels"] for p in all_semantic_points if not p["is_occluded"]]
            visible_labels = [p["label"] for p in all_semantic_points if not p["is_occluded"]]
            occluded_points_2d = [p["point_2d_pixels"] for p in all_semantic_points if p["is_occluded"]]
            occluded_labels = [p["label"] for p in all_semantic_points if p["is_occluded"]]
            
            if visible_points_2d:
                draw_points(img, np.array(visible_points_2d), labels=visible_labels, color=(0, 255, 0)) # Green
            if occluded_points_2d:
                draw_points(img, np.array(occluded_points_2d), labels=occluded_labels, color=(255, 0, 0)) # Red
            
            draw_points(img, corners_2d_pixels, labels=list(range(8)), color=(0, 0, 255))
            
            if hit_points_2d:
                draw_points(img, np.array(hit_points_2d), labels=hit_labels, color=(255, 255, 0), radius=3)

            self.backend.write_image(f"debug/{self.frame_id}.png", img)

        self.write_yolo_pose_format(all_semantic_points, corners_2d_pixels, width, height)
        
        self.frame_id += 1
        print("--- End of write method ---")

    def write_yolo_pose_format(self, semantic_points, corners_2d_pixels, width, height):
        """
        Generates a YOLO pose format annotation file.
        """
        x_min = np.min(corners_2d_pixels[:, 0])
        y_min = np.min(corners_2d_pixels[:, 1])
        x_max = np.max(corners_2d_pixels[:, 0])
        y_max = np.max(corners_2d_pixels[:, 1])
        
        bbox_center_x = (x_min + x_max) / 2
        bbox_center_y = (y_min + y_max) / 2
        bbox_width = x_max - x_min
        bbox_height = y_max - y_min

        bbox_center_x_norm = bbox_center_x / width
        bbox_center_y_norm = bbox_center_y / height
        bbox_width_norm = bbox_width / width
        bbox_height_norm = bbox_height / height

        yolo_line = f"0 {bbox_center_x_norm:.6f} {bbox_center_y_norm:.6f} {bbox_width_norm:.6f} {bbox_height_norm:.6f}"
        
        for point in semantic_points:
            x_norm = 0.0
            y_norm = 0.0
            visibility = 0
            
            if not point["is_occluded"]:
                x_norm = point["point_2d_pixels"][0] / width
                y_norm = point["point_2d_pixels"][1] / height
                visibility = 1
            
            yolo_line += f" {x_norm:.6f} {y_norm:.6f} {visibility}"
        
        file_path = f"{self.frame_id}.txt"
        self.backend.write_blob(file_path, yolo_line.encode())

rep.WriterRegistry.register(Semantic3dPointPoseEstimationWriter)
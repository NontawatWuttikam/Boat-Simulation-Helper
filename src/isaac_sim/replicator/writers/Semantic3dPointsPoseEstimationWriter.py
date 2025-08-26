import json
import omni.replicator.core as rep
from PIL import Image, ImageDraw
import omni.syntheticdata as sd
import numpy as np
from omni.physx import get_physx_scene_query_interface
from isaacsim.sensors.camera import Camera
import os
import copy

rep.settings.carb_settings("/omni/replicator/RTSubframes", 20)

"""
For the usage see README.md in the same folder
"""

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
	image_points[:, 1] = height - image_points[:, 1]
	for i, point in enumerate(image_points):
		xy = (point[0] - radius, point[1] - radius, point[0] + radius, point[1] + radius)
		draw.ellipse(xy, fill=color, outline=(0, 0, 0))
		if labels:
			draw.text((point[0] + radius, point[1] + radius), str(labels[i]), fill=color)


class Semantic3dPointPoseEstimationWriter(rep.Writer):
	def __init__(self, output_dir: str, target_prim: str = "/forklift_c", semantic_json_path: str = "/home/user/isaac_save/3d_model_semantic/forklift_c.json"):
		self.frame_id = 0
		self.backend = rep.BackendDispatch({"paths": {"out_dir": output_dir}})
		self.annotators = ["bounding_box_3d", "rgb", "camera_params"]
		self.raycast_distance_threshold = 0.4 #error between nearest hit and semantic point to consider occluded (to allow mildly occlusion, should not be 0 due to floating point error)
		self.target_prim = target_prim
		
		# Define the debug directory path and create it if it doesn't exist
		self.debug_output_dir = os.path.join(output_dir, "debug")
		os.makedirs(self.debug_output_dir, exist_ok=True)

		with open(semantic_json_path, "r") as f:
			self.semantic_data = json.load(f)

	def write(self, data):
		camera_prim_path = "/Replicator/Camera_Xform/Camera"
		self.camera = Camera(camera_prim_path, resolution=(1280, 720))
		
		# Create a copy of the raw RGB image before any annotations
		img = Image.fromarray(data["rgb"])
		img_original = copy.deepcopy(img)
		
		# Write the unannotated image to the main output directory
		self.backend.write_image(f"{self.frame_id}.png", img_original)
		
		render_product = [k for k in data.keys() if k.startswith("rp_")][0]
		width, height = data[render_product]["resolution"]

		bbox_info = data["bounding_box_3d"]["info"]
		prim_paths = bbox_info["primPaths"]
		
		try:
			target_index = prim_paths.index(self.target_prim)
		except ValueError:
			print(f"[Warning] Target prim '{self.target_prim}' not found! Skipping frame {self.frame_id}")
			return

		bbox3ds = data["bounding_box_3d"]["data"]
		corners_3d_all = sd.helpers.get_bbox_3d_corners(bbox3ds)
		corners_3d = corners_3d_all[target_index]

		bbl = corners_3d[0]
		ftr = corners_3d[7]
		bbox_size = ftr - bbl

		sem_norm = np.array([p["location"] for p in self.semantic_data["semantic_points"]])
		sem_name = np.array([p["name"] for p in self.semantic_data["semantic_points"]])
		sem_world = sem_norm * bbox_size + bbl

		sem_2d = world_to_image_pinhole(sem_world, data["camera_params"])
		sem_2d_pixels = sem_2d * np.array([[width, height]])

		corners_2d = world_to_image_pinhole(corners_3d, data["camera_params"])
		corners_2d_pixels = corners_2d * np.array([[width, height]])

		scene_query = get_physx_scene_query_interface()
		
		camera_origin, _ = self.camera.get_world_pose()
		camera_origin = np.array(camera_origin)
		
		# Unified list to store all semantic points and their occlusion state
		all_semantic_points = []
		
		# Lists for debug drawing of hit points
		hit_points_2d = []
		hit_labels = []

		for i, point_2d_pixels in enumerate(sem_2d_pixels):
			is_occluded = False
			
			ray_dir = sem_world[i] - camera_origin
			ray_dir /= np.linalg.norm(ray_dir)
			
			dist_to_sem_point = np.linalg.norm(sem_world[i] - camera_origin)

			hit = scene_query.raycast_closest(
				origin=camera_origin,
				dir=ray_dir,
				distance=1000.0
			)

			if hit:
				hit_pos_3d = hit['position']
				hit_pos_2d = world_to_image_pinhole(np.array([hit_pos_3d]), data["camera_params"])
				hit_pos_2d *= np.array([[width, height]])
				hit_points_2d.append(hit_pos_2d[0])
				hit_labels.append(sem_name[i])

				if hit['distance'] < dist_to_sem_point - self.raycast_distance_threshold:
					is_occluded = True
			
			all_semantic_points.append({
				"point_2d_pixels": point_2d_pixels,
				"label": sem_name[i],
				"is_occluded": is_occluded
			})

		# Check if the frame ID is within the first 5 frames to write annotated images
		if self.frame_id < 5:
			# Draw semantic points based on occlusion state
			visible_points_2d = [p["point_2d_pixels"] for p in all_semantic_points if not p["is_occluded"]]
			visible_labels = [p["label"] for p in all_semantic_points if not p["is_occluded"]]
			occluded_points_2d = [p["point_2d_pixels"] for p in all_semantic_points if p["is_occluded"]]
			occluded_labels = [p["label"] for p in all_semantic_points if p["is_occluded"]]

			if visible_points_2d:
				draw_points(img, np.array(visible_points_2d), labels=visible_labels, color=(0, 255, 0)) # Green
			if occluded_points_2d:
				draw_points(img, np.array(occluded_points_2d), labels=occluded_labels, color=(255, 0, 0)) # Red
			
			draw_points(img, corners_2d_pixels, labels=list(range(8)), color=(0, 0, 255))
			
			# Draw hit points in yellow
			if hit_points_2d:
				draw_points(img, np.array(hit_points_2d), labels=hit_labels, color=(255, 255, 0), radius=3)

			# Write the annotated image to the new debug subdirectory
			self.backend.write_image(f"debug/{self.frame_id}.png", img)
		
		# The YOLO pose format file will be written for all frames
		self.write_yolo_pose_format(all_semantic_points, corners_2d_pixels, width, height)
		
		self.frame_id += 1

	def write_yolo_pose_format(self, semantic_points, corners_2d_pixels, width, height):
		"""
		Generates a YOLO pose format annotation file.
		<class_id> <bbox_x_norm> <bbox_y_norm> <bbox_w_norm> <bbox_h_norm> <sem_1_x_norm> <sem_1_y_norm> <vis_1> ...
		"""
		x_min = np.min(corners_2d_pixels[:, 0])
		y_min = np.min(corners_2d_pixels[:, 1])
		x_max = np.max(corners_2d_pixels[:, 0])
		y_max = np.max(corners_2d_pixels[:, 1])
		
		bbox_center_x = (x_min + x_max) / 2
		bbox_center_y = (y_min + y_max) / 2
		bbox_width = x_max - x_min
		bbox_height = y_max - y_min

		bbox_center_y = height - bbox_center_y
		
		bbox_center_x_norm = bbox_center_x / width
		bbox_center_y_norm = bbox_center_y / height
		bbox_width_norm = bbox_width / width
		bbox_height_norm = bbox_height / height

		yolo_line = f"0\t{bbox_center_x_norm:.6f}\t{bbox_center_y_norm:.6f}\t{bbox_width_norm:.6f}\t{bbox_height_norm:.6f}"
		
		for point in semantic_points:
			x_norm = 0.0
			y_norm = 0.0
			visibility = 0
			
			if not point["is_occluded"]:
				x_norm = point["point_2d_pixels"][0] / width
				y_norm = (height - point["point_2d_pixels"][1]) / height
				visibility = 1
			
			yolo_line += f"\t{x_norm:.6f}\t{y_norm:.6f}\t{visibility}"
		
		file_path = f"{self.frame_id}.txt"
		self.backend.write_blob(file_path, yolo_line.encode())


rep.WriterRegistry.register(Semantic3dPointPoseEstimationWriter)
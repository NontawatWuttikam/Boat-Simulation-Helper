import json
import omni.replicator.core as rep
from PIL import Image, ImageDraw
import omni.syntheticdata as sd
import numpy as np
from omni.physx import get_physx_scene_query_interface
from isaacsim.sensors.camera import Camera

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
	def __init__(self, output_dir):
		self.frame_id = 0
		self.backend = rep.BackendDispatch({"paths": {"out_dir": output_dir}})
		self.annotators = ["bounding_box_3d", "rgb", "camera_params"]
		self.raycast_distance_threshold = 0.2 #error between nearest hit and semantic point to consider occluded (to allow mildly occlusion, should not be 0 due to floating point error)
		semantic_json_path = "/home/user/isaac_save/3d_model_semantic/forklift_c.json"
		
		with open(semantic_json_path, "r") as f:
			self.semantic_data = json.load(f)

	def write(self, data, target_prim="/forklift_c"):
		camera_prim_path = "/Replicator/Camera_Xform/Camera"
		self.camera = Camera(camera_prim_path, resolution=(1280, 720))
		img = Image.fromarray(data["rgb"])
		render_product = [k for k in data.keys() if k.startswith("rp_")][0]
		width, height = data[render_product]["resolution"]

		bbox_info = data["bounding_box_3d"]["info"]
		prim_paths = bbox_info["primPaths"]
		
		try:
			target_index = prim_paths.index(target_prim)
		except ValueError:
			print(f"[Warning] Target prim '{target_prim}' not found! Skipping frame {self.frame_id}")
			return

		bbox3ds = data["bounding_box_3d"]["data"]
		corners_3d_all = sd.helpers.get_bbox_3d_corners(bbox3ds)
		corners_3d = corners_3d_all[target_index] # This corner 3d is in world coordinates and consistent over every frame (Checked by me)

		print("corners_3d:", corners_3d)

		bbl = corners_3d[0]
		ftr = corners_3d[7]
		bbox_size = ftr - bbl

		sem_norm = np.array([p["location"] for p in self.semantic_data["semantic_points"]])
		sem_name = np.array([p["name"] for p in self.semantic_data["semantic_points"]])
		sem_world = sem_norm * bbox_size + bbl # this is also in world coord and consistent over every frame (Checked by me)

		print("Semantic points (world coordinates):", list(zip(sem_name, sem_world)))

		sem_2d = world_to_image_pinhole(sem_world, data["camera_params"])
		sem_2d *= np.array([[width, height]]) # this is also correctly projected (Check via plotting)

		corners_2d = world_to_image_pinhole(corners_3d, data["camera_params"])
		corners_2d *= np.array([[width, height]]) # this is also correctly projected (Check via plotting)

		scene_query = get_physx_scene_query_interface()
		
		camera_origin, _ = self.camera.get_world_pose()
		camera_origin = np.array(camera_origin)
		
		visible_points_2d = []
		visible_labels = []
		occluded_points_2d = []
		occluded_labels = []
		
		# New lists to store ray hit points and their labels
		hit_points_2d = []
		hit_labels = []

		print(f"\n--- DEBUG START: Frame {self.frame_id} ---")

		for i, point_2d in enumerate(sem_2d):
			point_name = self.semantic_data["semantic_points"][i]['name']
			print(f"\n- DEBUG Raycast for point: '{point_name}'")
			
			print("Raycasting variables:")
			
			ray_dir = sem_world[i] - camera_origin
			ray_dir /= np.linalg.norm(ray_dir)
			
			print(" 	DEBUG: Ray direction:", ray_dir)
			print(f" 	DEBUG: Semantic point (world): {sem_world[i]}")
			
			# Distance from camera to the semantic point
			dist_to_sem_point = np.linalg.norm(sem_world[i] - camera_origin)
			print(f" 	DEBUG: Distance to semantic point: {dist_to_sem_point}")
			
			# Print variables used in the raycast function
			print(f" 	DEBUG: Raycast function variables:")
			print(f" 	 	- origin: {camera_origin}")
			print(f" 	 	- dir: {ray_dir}")
			print(f" 	 	- distance: 1000.0")

			hit = scene_query.raycast_closest(
				origin=camera_origin,
				dir=ray_dir,
				distance=1000.0
			)

			# Check for occlusion
			if hit:
				print(f" 	DEBUG: Hit detected!")
				# Print all available hit information for detailed debugging
				print(f" 	 	- Hit info (full): {hit}")
				
				# Project the hit position to 2D image coordinates and add to the new list
				hit_pos_3d = hit['position']
				hit_pos_2d = world_to_image_pinhole(np.array([hit_pos_3d]), data["camera_params"])
				hit_pos_2d *= np.array([[width, height]])
				hit_points_2d.append(hit_pos_2d[0])
				hit_labels.append(point_name)

				# Use a small epsilon to avoid floating point issues
				if hit['distance'] < dist_to_sem_point - self.raycast_distance_threshold: 
					print(f" 	DEBUG: Point is occluded. Hit is closer than semantic point.")
					occluded_points_2d.append(point_2d)
					occluded_labels.append(point_name)
				else:
					print(f" 	DEBUG: Point is visible. Hit is farther than or at the same distance as the semantic point.")
					visible_points_2d.append(point_2d)
					visible_labels.append(point_name)
			else:
				print(f" 	DEBUG: No hit detected. Point is visible.")
				visible_points_2d.append(point_2d)
				visible_labels.append(point_name)
		
		print(f"\n--- DEBUG END: Frame {self.frame_id} ---")

		# Draw semantic points: visible points in green, occluded points in red
		if visible_points_2d:
			draw_points(img, np.array(visible_points_2d), labels=visible_labels, color=(0, 255, 0)) # Green
		if occluded_points_2d:
			draw_points(img, np.array(occluded_points_2d), labels=occluded_labels, color=(255, 0, 0)) # Red
		
		# Draw the new hit points in a different color (yellow)
		if hit_points_2d:
			draw_points(img, np.array(hit_points_2d), labels=hit_labels, color=(255, 255, 0), radius=2) # Yellow
		
		# Draw bbox corners (blue) + IDs
		corner_labels = list(range(8))
		draw_points(img, corners_2d, labels=corner_labels, color=(0, 0, 255))

		self.backend.write_image(f"{self.frame_id}.png", img)
		self.frame_id += 1


rep.WriterRegistry.register(Semantic3dPointPoseEstimationWriter)
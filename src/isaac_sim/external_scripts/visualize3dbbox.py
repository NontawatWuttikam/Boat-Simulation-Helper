import numpy as np
import cv2
import json

# -------------------------
# Config
# -------------------------
out_dir = "forklift_synthetic"
id = "0012"
target_prim = "/World/forklift_c"   # <--- specify your prim path here
# -------------------------

# Load all bboxes
bboxes = np.load(f"{out_dir}/bounding_box_3d_{id}.npy")

# Load prim paths
with open(f"{out_dir}/bounding_box_3d_prim_paths_{id}.json", "r") as f:
    prim_paths = json.load(f)

# Find index of the target prim
try:
    bbox_idx = prim_paths.index(target_prim)
except ValueError:
    raise RuntimeError(f"Target prim '{target_prim}' not found in {prim_paths}")

# Select the bbox
bbox = bboxes[bbox_idx]

# Load image
img = cv2.imread(f"{out_dir}/rgb_{id}.png")  # image for visualization

# Corners
x_min, y_min, z_min = bbox["x_min"], bbox["y_min"], bbox["z_min"]
x_max, y_max, z_max = bbox["x_max"], bbox["y_max"], bbox["z_max"]

corners = np.array([
    [x_min, y_min, z_min],
    [x_min, y_min, z_max],
    [x_min, y_max, z_min],
    [x_min, y_max, z_max],
    [x_max, y_min, z_min],
    [x_max, y_min, z_max],
    [x_max, y_max, z_min],
    [x_max, y_max, z_max],
], dtype=np.float32)

# Transform to world
T = bbox["transform"]
corners_h = np.hstack([corners, np.ones((8,1))])
corners_world = (T @ corners_h.T).T[:, :3]

# Load camera
with open(f"{out_dir}/camera_params_{id}.json", "r") as f:
    cam = json.load(f)

K = np.array([
    [1465.9985,    0.0,    640.0],
    [0.0,    1465.9985,    360.0],
    [0.0,       0.0,      1.0]
])

width, height = cam["renderProductResolution"]

View = np.array(cam["cameraViewTransform"]).reshape(4,4).T
R = View[:3,:3]
t = View[:3,3]

corners_cam = (R @ corners_world.T + t[:,None]).T
corners_cam[:, 0] *= -1

proj_corners, _ = cv2.projectPoints(corners_cam, np.zeros(3), np.zeros(3), K, None)
proj_corners = proj_corners.reshape(-1,2)

# Edges for visualization
edges = [
    (0,1),(0,2),(0,4),
    (3,1),(3,2),(3,7),
    (5,1),(5,4),(5,7),
    (6,2),(6,4),(6,7)
]

# Draw points and edges
for idx, pt in enumerate(proj_corners.astype(int)):
    pt_tuple = tuple(pt)
    cv2.circle(img, pt_tuple, 5, (0, 255, 0), -1)
    cv2.putText(img, str(idx), (pt_tuple[0]+5, pt_tuple[1]-5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)

for i, j in edges:
    pt1, pt2 = tuple(proj_corners[i].astype(int)), tuple(proj_corners[j].astype(int))
    cv2.line(img, pt1, pt2, (0, 255, 255), 2)

cv2.imshow("Projected 3D BBox", img)
cv2.waitKey(0)
cv2.destroyAllWindows()

import bpy
import numpy as np
import json
import os

# Back-Bottom-Left (id 0)
# Front-Top-Right (id 7)
bbox_corners = [
    (-1.8915, -0.6951, 0.0025),
    (-1.8922, -0.7044, 2.1816),
    (-1.8998, 0.6837, -0.0001),
    (-1.8933, 0.6885, 2.1807),
    (1.8834, -0.6994, 0.0015),
    (1.8836, -0.6967, 2.1800),
    (1.8812, 0.6830, 0.0049),
    (1.8835, 0.6867, 2.1851)
]

# Semantic point labels
forklift_labels = [
    "Left fork tip",
    "Right fork tip",
    "Front left roof",
    "Front right roof",
    "Back left roof",
    "Back right roof",
    "Front left wheel",
    "Front right wheel",
    "Back left wheel",
    "Back right wheel"
]

# Forklift points
forklift_points = [
    (1.8682, -0.2965, 0.1361),
    (1.8586, 0.2941, 0.1417),
    (-0.1237, -0.4671, 2.1121),
    (-0.1202, 0.4558, 2.1211),
    (-1.1013, -0.4875, 2.1177),
    (-1.1142, 0.4933, 2.1204),
    (0.2611, -0.6544, 0.0047),
    (0.2655, 0.6542, 0.0091),
    (-1.3864, -0.6846, 0.0135),
    (-1.3863, 0.6756, 0.0057)
]


bbox_corners_np = np.array(bbox_corners)
forklift_points_np = np.array(forklift_points)

bbl_corner = bbox_corners_np[0]
ftr_corner = bbox_corners_np[7]

bbox_size = ftr_corner - bbl_corner

forklift_points_normalized = (forklift_points_np - bbl_corner) / bbox_size

data = {
    "model_name": "forklift_c",
    "description": "isaac sim forklift_c semantic points normalized into [0,1] bbox space (BBL=0,0,0 / FTR=1,1,1)",
    "semantic_points": []
}

for i, (name, pt) in enumerate(zip(forklift_labels, forklift_points_normalized)):
    data["semantic_points"].append({
        "id": i,
        "name": name,
        "location": pt.tolist()
    })

out_path = "D:\\AI-Safety\\3d model semantic\\forklift_c_normalized.json"
with open(out_path, "w") as f:
    json.dump(data, f, indent=2)

print(f"Saved {out_path}")

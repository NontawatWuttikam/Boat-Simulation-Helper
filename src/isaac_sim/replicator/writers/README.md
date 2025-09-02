# 📌 **Semantic3dPointPoseEstimationWriter: A Guide to Usage**

This guide explains how to use the custom **Semantic3dPointPoseEstimationWriter** to capture **RGB images** 🎥 and **3D semantic points** 📐 of a target object, like a forklift 🚜.  
The process involves three main stages:  

1. Preparing your data 🗂️  
2. Defining semantic points in Blender 🎨  
3. Generating the final dataset in Isaac Sim 📊  

---

## 🔹 **Pre-step: Preparing Your 3D Bounding Box Data**

Before defining your semantic points, you need to prepare the **3D bounding box** information for your target object.

1. 📥 **Import** your target object's USD (e.g., forklift) into Isaac Sim.  
   👉 Don’t forget to add the semantic label via the **Semantic Schema Editor** panel.  
   *(Click the target prim in the stage panel → click **Add Entries On All Selected Prim** in Semantic Schema Editor)*  

2. 🎬 Use the **Synthetic Data Recorder** to export the object's:  
   - `3d_bbox`  
   - `camera_params`

3. 🖼️ Run the script **`visualize3dbbox.py`** (found in `src/isaac_sim/external_scirpts/`) with:  
   - The output directory from step 2  
   - The target prim path  

4. 📊 The script will generate a **plot showing the specific corner order** of the 3D bounding box.  
   👉 You’ll need this order later.  

---

## 🎨 **Blender: Defining Semantic Points**

This step uses **Blender** to define semantic points on your 3D model.

1. 📥 **Import** the target USD from the previous step into Blender.  
2. 🔲 Create a **bounding box** enclosing the model.  
3. 🎯 Use the **3D cursor** to click on a **corner of the bounding box**.  
   - Follow the **same corner order** identified in the *Pre-step*.  
4. ▶️ Run the script: **`src/isaac_sim/blender/print_3d_cursor.py`**.  
5. 📋 Copy the **3D cursor’s coordinates** → paste them into:  
   **`src/isaac_sim/blender/Semantic3dPointsFileCreator.py`**  
6. 🔁 Repeat **steps 3–5** for **all 8 bounding box corners**, maintaining the correct order.  
7. 👻 Hide the **bounding box** in Blender.  
8. 📍 Use the **3D cursor** again to click on the **semantic points** you want on the model itself (e.g., wheels, roof, fork tips 🚜).  
   - Repeat the process as with the bounding box corners.  
   - Specify a **name** for each semantic point in `Semantic3dPointsFileCreator.py`.  
9. ▶️ Copy the entire, **edited code** from `Semantic3dPointsFileCreator.py` → **run it in Blender**.  
10. 📑 This generates a **JSON file** containing semantic point data.  
    ⚠️ **Note:** The coordinates are **relative to the origin** of the bounding box (the **first bounding box point** you defined).  

---

## 📊 **Isaac Sim: Generating the Data**

This is the final step: using the generated **JSON file** and the custom writer to create your dataset in Isaac Sim.

⚠️ **Important Preparations**  

- 🛑 Ensure the target object **does not move or rotate** during simulation.  
  - If needed, **disable physics** to prevent inconsistencies in 3D bounding box corner orders.  
- ⚙️ For `Semantic3dPointPoseEstimationWriter` to work:  
  - ❌ Remove **articulation root** and **physics attributes** (collider, rigid body, etc.) from the target object.  
  - Otherwise, raycasting will misalign (e.g., computing at timestep *t-1* while images are from timestep *t*, causing **weird occlusion issues**).  

---

### 🎥 Camera Setup

1. 🎲 To create a **random replicator camera**, run:  
   **`snippets/createRandomViewCameraReplicator.py`**  
   - Change the **lookat parameter** to your target prim.  
   - Modify the **pose distribution** according to your dataset requirements.  
   - 👉 If you want to use a **fixed camera**, you can skip this step.  

2. ✅ Ensure that the **camera prim path** in your `Semantic3dPointPoseEstimationWriter` code is **correct**.  

---

### 🚀 Steps in Isaac Sim

1. 📜 Copy the code from **`Semantic3dPointPoseEstimationWriter.py`** into Isaac Sim’s **Script Editor**.  
2. 📂 In the writer class, specify the **path to the JSON file** you created and **camera_prim_path** and **target_prim**.  
3. ⚙️ Adjust parameters as needed:  
   - Raycast threshold  
4. ▶️ Run the script to **register the writer**.  
5. In the **Synthetic Data Recorder** panel:  
   - 🎥 Set the **Camera Path** to your Replicator camera prim path.  
   - ✏️ Under **Parameter**, change **Writer** from `default` → `Semantic3dPointPoseEstimationWriter`.  
   - 📂 Specify the **Output Dataset Path** and the number of images to generate.  
6. ⚠️ **IMPORTANT:** Press **Play ▶️** to start the simulation.  
   - **Physics must be running** for raycasting to work correctly.  
7. 🎬 Click **Start** in the Synthetic Data Recorder panel.  
8. 📑 After completion, check your output directory:  

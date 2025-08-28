---

## **Semantic3dPointPoseEstimationWriter: A Guide to Usage**

This guide explains how to use the custom **Semantic3dPointPoseEstimationWriter** to capture RGB images and 3D semantic points of a target object, like a forklift. This process involves a few key steps: preparing your data, defining semantic points in Blender, and generating the final dataset in Isaac Sim.

---

### **Pre-step: Preparing Your 3D Bounding Box Data**

Before defining your semantic points, you need to prepare the 3D bounding box information for your target object.

1.  **Import** your target object's USD (e.g., forklift) into Isaac Sim. Don't forget to add the semantic via Semantic Schema Editor panel. (click the target prim in the stage panel and click Add Entries On All Selected Prim in Semantic Schema Editor)
2.  Use the **Synthetic Data Recorder** to export the object's **`3d_bbox`** and **`camera_params`**.
3.  Run the **`visualize3dbbox.py`** script (located in `src/isaac_sim/external_scirpts/`) and specify the output directory from the previous step and the target prim.
4.  The script will generate a plot that shows the specific corner order of the 3D bounding box. You'll need this order later.

---

### **Blender: Defining Semantic Points** 🎨

This part of the process uses Blender to define the semantic points on your 3D model.

1.  **Import** the target USD from the previous step into Blender.
2.  Create a bounding box that encloses the model.
3.  Use the **3D cursor** to click on a corner of the bounding box. Make sure you follow the same corner order that you identified in the previous "Pre-step."
4.  Run the script located at **`src/isaac_sim/blender/print_3d_cursor.py`**.
5.  Copy the 3D cursor's coordinates and paste them into the script at **`src/isaac_sim/blender/Semantic3dPointsFileCreator.py`**.
6.  Repeat steps 3-5 for all 8 bounding box corners, making sure to maintain the correct order.
7.  Once the bounding box corners are defined, **hide the bounding box** in Blender.
8.  Now, use the 3D cursor to click on the semantic points you want to define on the 3D model itself (e.g., wheels, roof, or fork tips for a forklift). Repeat the same process as with the bounding box corners, and specify the name for each semantic point in the `Semantic3dPointsFileCreator.py` script.
9.  Finally, copy the entire, edited code from the `Semantic3dPointsFileCreator.py` script and run it in Blender.
10. This will generate a JSON file containing the semantic point data. **Note:** The coordinates in this file are relative to the origin of the bounding box (the first bounding box point you defined).

---

### **Isaac Sim: Generating the Data** 📊

The final step is to use the generated JSON file and the custom writer to create your dataset in Isaac Sim.

0   **Important** Make sure the target object didn't move, rotate from external force while playing simulation, or safely disable its physics attribute. OTHERWIESE the 3d bbox corner orders might be inconsistent.
1.  Copy the code from **`Semantic3dPointPoseEstimationWriter.py`** into Isaac Sim's Script Editor.
2.  In the writer class, specify the path to the JSON file you just created.
3.  Adjust any necessary parameters, such as the target prim path, raycast threshold, or Replicator camera prim path.
4.  Run the script to register the writer.
5.  In the **Synthetic Data Recorder** panel:
    * Set the **Camera Path** to your Replicator camera prim path.
    * Under the **Parameter** section, change the **Writer** from `default` to `custom`.
    * Set the **Writer Name** to `Semantic3dPointPoseEstimationWriter`.
    * Specify the **Output Dataset Path** and the number of images you want to generate.
6.  **"IMPORTANT!!"** Press the **"Play"** button to start the simulation. **Physics must be running** for the raycast to work correctly.
7.  Now, click **"Start"** in the Synthetic Data Recorder panel.
8.  After the process is complete, check your output directory (`<output_dir>/example`) to confirm that the semantic points were correctly projected onto the images. 
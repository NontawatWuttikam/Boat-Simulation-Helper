from isaacsim.examples.interactive.base_sample import BaseSample
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.prims import XFormPrim
import numpy as np
import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.utils.stage import open_stage_async, open_stage
from omni.isaac.core import World
from isaacsim.core.api.physics_context import PhysicsContext
enable_extension("omni.kit.scripting")
import asyncio
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.api.robots import Robot
from pxr import Usd, UsdPhysics, Sdf
from isaacsim.core.api import SimulationContext
from isaacsim.core.api.physics_context import PhysicsContext
from omni.isaac.sensor import Camera
import isaacsim.replicator.agent.core.simulation as agentsim
from isaacsim.replicator.agent.core.config_file.util import ConfigFileUtil
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.core.utils.types import ArticulationAction
from omni.isaac.core.articulations import ArticulationView
import cv2
import matplotlib.pyplot as plt
import omni
import os
import carb

OMNI_ANIM_PEOPLE_COMMAND_PATH = "/exts/omni.anim.people/command_settings/command_file_path"

class RecordMultiCam(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        self.video_writers = []  # List for each cam's VideoWriter
    
    def setup_scene(self):
        return
    
    def setup_robot(self):
        add_reference_to_stage(usd_path="https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Robots/Forklift/forklift_c.usd", prim_path="/Forklift")
        self.sim_ctx.scene.add(Robot(prim_path="/Forklift", name="forklift",))
        self.forklift = self.sim_ctx.scene.get_object("forklift")
        print("get articulation")
    
    def drive_robot(self):
        self.forklift.apply_action(ArticulationAction(joint_positions=np.array([0, 0, 0, 0, 0, 0, 0]),
                                                                            joint_efforts=np.array([0, 0, 0, 0, 0, 0, 0]),
                                                                            joint_velocities = np.array([10, 10, 10, 10, 0, 0, 0])))
        
    def setup_people(self):
        self.agent_manager = agentsim.SimulationManager()
        default_config_file_path = ConfigFileUtil.get_default_config_file_path()
        print("default_config_file_path", default_config_file_path)
        self.agent_manager.load_config_file(default_config_file_path)
        self.agent_manager.spawn_character_by_idx(np.array([-2.6,25.4,0.0]), 0, 0)
        self.agent_manager.setup_all_characters()
        
        # agents_pos = self.agent_manager._get_all_agents_positions()
        # self.agent_manager._character_randomizer.update_agent_positions(agents_pos)
        
        command = [
            "Character GoTo -2.6 0.53 0.0 _",
            # "Character LookAround 3.41",
            # "Character GoTo 7.01 -0.18 0.05 _",
            # "Character LookAround 2.58",
            # "Character Idle 2.81",
            # "Character GoTo -5.51 -6.86 0.05 _",
            # "Character Idle 5.77",
            # "Character LookAround 2.89",
            # "Character GoTo 3.05 8.95 0.05 _"
        ]
        
        # command = self.agent_manager.generate_random_commands()
        # print("generate_random_commands", command)
        
        temp_command_path = "temp_omni_people_command.txt"
        with open(temp_command_path, "w") as f:
            f.write("\n".join(command))
        carb.settings.get_settings().set(OMNI_ANIM_PEOPLE_COMMAND_PATH, temp_command_path)
        # spawn_character_by_idx, setup_all_characters
    
    async def setup_post_load(self):
        self.target_fps = 30
        self.duration = 60 # in second
        self.resolution = (1280, 720)
        self.save_video_path = "/home/user/isaac_save/export_videos/test2"
        # self.stage = "/home/user/isaac_save/multicam_people_walking.usd"
        # self.stage = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"
        # self.stage = "/home/user/isaac_save/Environments/full_warehouse_with_cameras.usd"
        self.stage = "/home/user/isaac_save/small_plane.usd"
        
        self.interval = 1.0 / self.target_fps
        self.elapsed_time = 0
        self.accum_time = 0
        self.cameras = []
        
        if not os.path.exists(self.save_video_path):
            os.makedirs(self.save_video_path)
        open_stage(self.stage)
        
        
        PhysicsContext_instance = PhysicsContext(prim_path="/World/PhysicsScene")
        simulation_context = World()
        simulation_context._physics_context = PhysicsContext_instance
        self.sim_ctx = simulation_context
        
        # self.sim_ctx.scene.add_default_ground_plane()
        self.setup_people()
        self.setup_robot()
        
        self.sim_ctx.add_physics_callback("sim_step", self.physics_callback)
        
        # camera_list = ["/World/Cameras/Camera_01", "/World/Cameras/Camera_02"]
        # for camera_path in camera_list:
        #     camera = Camera(prim_path=camera_path, resolution = self.resolution)
        #     camera.initialize()
        #     self.cameras.append(camera)
        
        # self.video_writers = []
        # for idx, cam in enumerate(self.cameras):
        #     width, height = self.resolution
        #     fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        #     filename = os.path.join(self.save_video_path, f'output_cam_{idx+1}.mp4')
        #     writer = cv2.VideoWriter(filename, fourcc, self.target_fps, (width, height))
        #     self.video_writers.append(writer)
    
    def record(self, sim_step):
        self.elapsed_time += sim_step
        if self.elapsed_time >= self.interval:
            for idx, cam in enumerate(self.cameras):
                frame = cam.get_rgb()
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                # cv2.imshow(f"camera_{idx+1}", frame)
                # cv2.waitKey(1)
                self.video_writers[idx].write(frame)
            self.elapsed_time -= self.interval
        self.accum_time += sim_step
        if self.accum_time >= self.duration:
            self.release_video_writers()
            self.sim_ctx.stop()
        return     
    
    def physics_callback(self, sim_step):
        print("physic callback is working")
        # self.record(sim_step)
        self.drive_robot()
        
    
    def release_video_writers(self):
        for writer in self.video_writers:
            writer.release()
        cv2.destroyAllWindows()
    
    def on_stop(self):
        self.release_video_writers()
        print("resetrtttt")
        
        
    async def setup_post_reset(self):
        print("resetrtttt")
        return
        
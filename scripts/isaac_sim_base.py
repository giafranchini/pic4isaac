from omni.isaac.core import World
from omni.isaac.core.utils import nucleus, stage, prims
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.core.utils.types import ArticulationAction

import carb
import yaml
import sys
import numpy as np
import os
from collections import namedtuple

robot_namedtuple = namedtuple("Robot", ["robot_object", "cameras", "lidar"])

class IsaacBase():
    def __init__(self, simulation_app):
        self.__simulation_app = simulation_app
        self.__simulation_context = World()
        self.root_prim = str((prims.create_prim(prim_path="/World/map")).GetPath())
        self.update()
        self.robots = [] # robots is a list of namedtuple (robot [WheeledRobot object], cameras [dict], lidar [dict])

    def __get_assets_root_path(self):
        assets_root_path = nucleus.get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            self.__simulation_app.close()
            sys.exit()
        return assets_root_path
    
    def start(self):
        self.__simulation_context.initialize_physics()
        self.__simulation_context.play()
        for r in self.get_robot_assets():
            # Workaround for non 0 velocities at start
            r_obj = self.__simulation_context.scene.get_object(r.robot_object.name)
            r_obj.apply_wheel_actions(
                ArticulationAction(joint_positions=None, joint_efforts=None, joint_velocities=np.zeros(4)))

    def stop(self):
        self.__simulation_context.stop()
    
    def update(self):
        self.__simulation_app.update()

    def add_default_ground_plane(self):
        self.__simulation_context.scene.add_default_ground_plane()
        self.__simulation_app.update()
    
    def add_usd_to_stage(self, usd_path : str, root_prim_path : str):
        # add as child of world as default
        # NB: USD needs to have a default prim 
        prim_path = os.path.join(self.root_prim, root_prim_path)
        stage.add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)

    def spawn_robots(self, robot_config : dict):
        # Only wheeled robots are supported right now
        config_file = robot_config
        for r_conf in config_file.values():
            robot_path = r_conf.get("isaac_asset_path")
            robot_name = r_conf.get("name")
            # print(f"Spawning robot: {robot_name} at path {robot_path}")
            self.add_usd_to_stage(usd_path=robot_path, root_prim_path=self.root_prim)
            robot_obj = self.__simulation_context.scene.add(
                WheeledRobot(
                prim_path=f"{self.root_prim}/{robot_name}",
                name=robot_name,
                wheel_dof_names=r_conf.get("wheel_joint_names"),
                create_robot=True,
                usd_path=robot_path,
                position=np.array(r_conf.get("position")),
                orientation=np.array(r_conf.get("orientation")),
                ))
            cameras = r_conf.get("cameras")
            lidar = r_conf.get("lidar")
            self.robots.append(robot_namedtuple(robot_obj, cameras, lidar))

    def get_robot_assets(self):
        return self.robots

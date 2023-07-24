from isaac_sim_base import IsaacBase
import omni.graph.core as og
from omni.isaac.core_nodes.scripts.utils import set_target_prims
from omni.isaac.core.utils import stage, prims

class IsaacRos(IsaacBase):
    def __init__(self, simulation_app, physics_rate):
        super().__init__(simulation_app=simulation_app, physics_rate=physics_rate)

    def __init_controller(self):
            self.gc = og.Controller()
            self.graph_dict = {
            self.gc.Keys.CREATE_NODES: [],
            self.gc.Keys.CONNECT: [],
            self.gc.Keys.SET_VALUES: [],
            }

    def __create_ros_control_nodes(self, robot):
        self.graph_dict[self.gc.Keys.CREATE_NODES] = ([
            ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
            ("ros2_context", "omni.isaac.ros2_bridge.ROS2Context"),
            ("read_sim_time", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
            ("ros2_publish_clock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
            ("ros2_publish_joint_states", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
            ("ros2_subscribe_joint_states", "omni.isaac.ros2_bridge.ROS2SubscribeJointState"),
            ("node_namespace", "omni.graph.nodes.ConstantString"),
            ("isaac_articulation_controller", "omni.isaac.core_nodes.IsaacArticulationController"),
            ("ros2_publish_tf", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
        ])
        self.graph_dict[self.gc.Keys.CONNECT] = ([
            ("on_playback_tick.outputs:tick", "ros2_publish_clock.inputs:execIn"),
            ("ros2_context.outputs:context", "ros2_publish_clock.inputs:context"),
            ("read_sim_time.outputs:simulationTime", "ros2_publish_clock.inputs:timeStamp"),
            ("on_playback_tick.outputs:tick", "ros2_subscribe_joint_states.inputs:execIn"),
            ("on_playback_tick.outputs:tick", "ros2_publish_joint_states.inputs:execIn"),
            ("ros2_context.outputs:context", "ros2_subscribe_joint_states.inputs:context"),
            ("ros2_context.outputs:context", "ros2_publish_joint_states.inputs:context"),
            ("ros2_subscribe_joint_states.outputs:execOut", "isaac_articulation_controller.inputs:execIn"),
            ("ros2_subscribe_joint_states.outputs:jointNames", "isaac_articulation_controller.inputs:jointNames"),
            ("ros2_subscribe_joint_states.outputs:velocityCommand", "isaac_articulation_controller.inputs:velocityCommand"),
            ("node_namespace.inputs:value", "ros2_subscribe_joint_states.inputs:nodeNamespace"),
            ("node_namespace.inputs:value", "ros2_publish_joint_states.inputs:nodeNamespace"),
            ("on_playback_tick.outputs:tick", "ros2_publish_tf.inputs:execIn"),
            ("ros2_context.outputs:context", "ros2_publish_tf.inputs:context"),
            ("read_sim_time.outputs:simulationTime", "ros2_publish_tf.inputs:timeStamp"),
        ])
        self.graph_dict[self.gc.Keys.SET_VALUES] = ([
            ("ros2_context.inputs:domain_id", 89),
            ("ros2_subscribe_joint_states.inputs:topicName", "/isaac_joint_commands"),
            ("ros2_publish_joint_states.inputs:topicName", "/isaac_joint_states"),
            ("isaac_articulation_controller.inputs:usePath", True),
            ("isaac_articulation_controller.inputs:robotPath", robot.prim_path),
        ])
    
    def __create_cameras_nodes(self, robot, cameras):

        viewport_id = 0
        self.graph_dict[self.gc.Keys.CREATE_NODES] = ([
            ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
            ("ros2_context", "omni.isaac.ros2_bridge.ROS2Context"),
            ("read_sim_time", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
        ])
        self.graph_dict[self.gc.Keys.SET_VALUES] = ([
            ("ros2_context.inputs:domain_id", 89),
        ])
        for cam_name, cam_conf in cameras.items():
            rgb_condition = cam_conf.get("enable_rgb")
            depth_condition = cam_conf.get("enable_depth")
            self.graph_dict[self.gc.Keys.CREATE_NODES].extend([
                (f"viewport_id_{cam_name}", "omni.graph.nodes.ConstantUInt"),
                (f"isaac_create_viewport_{cam_name}", "omni.isaac.core_nodes.IsaacCreateViewport"),
                (f"isaac_set_viewport_res_{cam_name}", "omni.isaac.core_nodes.IsaacSetViewportResolution"),
                (f"isaac_get_viewport_render_product_{cam_name}", "omni.isaac.core_nodes.IsaacGetViewportRenderProduct"),
                (f"isaac_set_{cam_name}", "omni.isaac.core_nodes.IsaacSetCameraOnRenderProduct"),
                (f"ros2_create_{cam_name}_info", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                (f"ros2_create_{cam_name}_rgb", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                (f"ros2_create_{cam_name}_depth", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                (f"enable_{cam_name}_rgb", "omni.graph.action.Branch"),
                (f"enable_{cam_name}_depth", "omni.graph.action.Branch"),
                (f"{cam_name}_rgb_gate", "omni.isaac.core_nodes.IsaacSimulationGate"),
                (f"{cam_name}_depth_gate", "omni.isaac.core_nodes.IsaacSimulationGate"),
            ])
            self.graph_dict[self.gc.Keys.CONNECT].extend([
                ("on_playback_tick.outputs:tick", f"isaac_create_viewport_{cam_name}.inputs:execIn"),
                ("on_playback_tick.outputs:tick", f"isaac_set_viewport_res_{cam_name}.inputs:execIn"),
                (f"isaac_create_viewport_{cam_name}.outputs:execOut", f"isaac_get_viewport_render_product_{cam_name}.inputs:execIn"),
                (f"isaac_create_viewport_{cam_name}.outputs:viewport", f"isaac_get_viewport_render_product_{cam_name}.inputs:viewport"),
                (f"isaac_create_viewport_{cam_name}.outputs:viewport", f"isaac_set_viewport_res_{cam_name}.inputs:viewport"),
                (f"isaac_get_viewport_render_product_{cam_name}.outputs:execOut", f"isaac_set_{cam_name}.inputs:execIn"),
                (f"isaac_get_viewport_render_product_{cam_name}.outputs:renderProductPath", f"isaac_set_{cam_name}.inputs:renderProductPath"),
                (f"isaac_get_viewport_render_product_{cam_name}.outputs:renderProductPath", f"ros2_create_{cam_name}_info.inputs:renderProductPath"),
                (f"isaac_get_viewport_render_product_{cam_name}.outputs:renderProductPath", f"ros2_create_{cam_name}_rgb.inputs:renderProductPath"),
                (f"isaac_get_viewport_render_product_{cam_name}.outputs:renderProductPath", f"ros2_create_{cam_name}_depth.inputs:renderProductPath"),
                (f"isaac_set_{cam_name}.outputs:execOut", f"ros2_create_{cam_name}_info.inputs:execIn"),
                (f"isaac_set_{cam_name}.outputs:execOut", f"enable_{cam_name}_rgb.inputs:execIn"),
                (f"isaac_set_{cam_name}.outputs:execOut", f"enable_{cam_name}_depth.inputs:execIn"),
                (f"enable_{cam_name}_rgb.outputs:execTrue", f"{cam_name}_rgb_gate.inputs:execIn"),
                (f"{cam_name}_rgb_gate.outputs:execOut", f"ros2_create_{cam_name}_rgb.inputs:execIn"),
                (f"enable_{cam_name}_depth.outputs:execTrue", f"{cam_name}_depth_gate.inputs:execIn"),
                (f"{cam_name}_depth_gate.outputs:execOut", f"ros2_create_{cam_name}_depth.inputs:execIn"),
                ("ros2_context.outputs:context", f"ros2_create_{cam_name}_info.inputs:context"),
                ("ros2_context.outputs:context", f"ros2_create_{cam_name}_rgb.inputs:context"),
                ("ros2_context.outputs:context", f"ros2_create_{cam_name}_depth.inputs:context"),
            ])
            self.graph_dict[self.gc.Keys.SET_VALUES].extend([
                (f"isaac_create_viewport_{cam_name}.inputs:viewportId", viewport_id),
                (f"enable_{cam_name}_rgb.inputs:condition", rgb_condition),
                (f"enable_{cam_name}_depth.inputs:condition", depth_condition), 
                (f"ros2_create_{cam_name}_info.inputs:frameId", cam_conf.get("frame_id")),
                (f"ros2_create_{cam_name}_rgb.inputs:frameId", cam_conf.get("frame_id")),
                (f"ros2_create_{cam_name}_depth.inputs:frameId", cam_conf.get("frame_id")),
                (f"ros2_create_{cam_name}_info.inputs:nodeNamespace", f"/{robot.name}/{cam_name}"),
                (f"ros2_create_{cam_name}_rgb.inputs:nodeNamespace", f"/{robot.name}/{cam_name}"),
                (f"ros2_create_{cam_name}_depth.inputs:nodeNamespace", f"/{robot.name}/{cam_name}"),
                (f"ros2_create_{cam_name}_info.inputs:topicName", "camera_info"),
                (f"ros2_create_{cam_name}_rgb.inputs:topicName", "rgb"),
                (f"ros2_create_{cam_name}_depth.inputs:topicName", "depth"),
                (f"ros2_create_{cam_name}_info.inputs:type", "camera_info"),
                (f"ros2_create_{cam_name}_rgb.inputs:type", "rgb"),
                (f"ros2_create_{cam_name}_depth.inputs:type", "depth"),
                (f"viewport_id_{cam_name}.inputs:value", viewport_id),
                (f"isaac_create_viewport_{cam_name}.inputs:name", cam_name),
                (f"isaac_set_viewport_res_{cam_name}.inputs:width", 640),
                (f"isaac_set_viewport_res_{cam_name}.inputs:height", 480),
                (f"{cam_name}_rgb_gate.inputs:step", 5),
                (f"{cam_name}_depth_gate.inputs:step", 60),
            ])
            viewport_id += 1
    
    def __create_lidar_nodes(self, robot, lidar_conf):
        self.graph_dict[self.gc.Keys.CREATE_NODES] = ([
            ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
            ("ros2_context", "omni.isaac.ros2_bridge.ROS2Context"),
            ("read_sim_time", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
            ("laser_scan_reader", "omni.isaac.range_sensor.IsaacReadLidarBeams"),
            ("pcl_reader", "omni.isaac.range_sensor.IsaacReadLidarPointCloud"),
            ("laser_scan_publisher", "omni.isaac.ros2_bridge.ROS2PublishLaserScan"),
            ("pcl_publisher", "omni.isaac.ros2_bridge.ROS2PublishPointCloud"),
            ("enable_scan", "omni.graph.action.Branch"),
            ("enable_pcl", "omni.graph.action.Branch"),
        ])
        self.graph_dict[self.gc.Keys.CONNECT] = ([
            ("on_playback_tick.outputs:tick", "enable_scan.inputs:execIn"),
            ("enable_scan.outputs:execTrue", "laser_scan_reader.inputs:execIn"),
            ("ros2_context.outputs:context", "laser_scan_publisher.inputs:context"),
            ("laser_scan_reader.outputs:execOut", "laser_scan_publisher.inputs:execIn"),
            ("laser_scan_reader.outputs:horizontalFov", "laser_scan_publisher.inputs:horizontalFov"),
            # ("laser_scan_reader.outputs:verticalFov", "laser_scan_publisher.inputs:verticalFov"),
            ("laser_scan_reader.outputs:horizontalResolution", "laser_scan_publisher.inputs:horizontalResolution"),
            # ("laser_scan_reader.outputs:verticalResolution", "laser_scan_publisher.inputs:verticalResolution"),
            ("laser_scan_reader.outputs:depthRange", "laser_scan_publisher.inputs:depthRange"),
            ("laser_scan_reader.outputs:rotationRate", "laser_scan_publisher.inputs:rotationRate"),
            ("laser_scan_reader.outputs:linearDepthData", "laser_scan_publisher.inputs:linearDepthData"),
            ("laser_scan_reader.outputs:intensitiesData", "laser_scan_publisher.inputs:intensitiesData"),
            ("laser_scan_reader.outputs:numRows", "laser_scan_publisher.inputs:numRows"),
            ("laser_scan_reader.outputs:numCols", "laser_scan_publisher.inputs:numCols"),
            ("laser_scan_reader.outputs:azimuthRange", "laser_scan_publisher.inputs:azimuthRange"),
            # ("laser_scan_reader.outputs:zenithRange", "laser_scan_publisher.inputs:zenithRange"),
            ("read_sim_time.outputs:simulationTime", "laser_scan_publisher.inputs:timeStamp"),
            ("on_playback_tick.outputs:tick", "enable_pcl.inputs:execIn"),
            ("enable_pcl.outputs:execTrue", "pcl_reader.inputs:execIn"),
            ("pcl_reader.outputs:execOut", "pcl_publisher.inputs:execIn"),
            ("ros2_context.outputs:context", "pcl_publisher.inputs:context"),
            ("pcl_reader.outputs:pointCloudData", "pcl_publisher.inputs:pointCloudData"),
            ("read_sim_time.outputs:simulationTime", "pcl_publisher.inputs:timeStamp"),
        ])
        self.graph_dict[self.gc.Keys.SET_VALUES] = ([
            ("ros2_context.inputs:domain_id", 89),
            ("laser_scan_publisher.inputs:nodeNamespace", f"/{robot.name}"),
            ("laser_scan_publisher.inputs:topicName", "scan"),
            ("laser_scan_publisher.inputs:frameId", "front_laser"),
            ("pcl_publisher.inputs:nodeNamespace", f"/{robot.name}"),
            ("pcl_publisher.inputs:topicName", "points"),
            ("pcl_publisher.inputs:frameId", "front_laser"),
            ("enable_scan.inputs:condition", lidar_conf.get("enable_scan")),
            ("enable_pcl.inputs:condition", lidar_conf.get("enable_pcl")),
        ])
    
    def __create_imu_nodes(self, robot, imu):
        self.graph_dict[self.gc.Keys.CREATE_NODES] = ([
            ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
            ("ros2_context", "omni.isaac.ros2_bridge.ROS2Context"),
            ("read_sim_time", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
            ("imu_reader", "omni.isaac.sensor.IsaacReadIMU"),
            ("imu_publisher", "omni.isaac.ros2_bridge.ROS2PublishImu"),
        ])
        self.graph_dict[self.gc.Keys.CONNECT] = ([
            ("on_playback_tick.outputs:tick", "imu_reader.inputs:execIn"),
            ("imu_reader.outputs:execOut", "imu_publisher.inputs:execIn"),
            ("read_sim_time.outputs:simulationTime", "imu_publisher.inputs:timeStamp"),
            ("ros2_context.outputs:context", "imu_publisher.inputs:context"),
            ("imu_reader.outputs:angVel", "imu_publisher.inputs:angularVelocity"),
            ("imu_reader.outputs:linAcc", "imu_publisher.inputs:linearAcceleration"),
            ("imu_reader.outputs:orientation", "imu_publisher.inputs:orientation"),
        ])
        self.graph_dict[self.gc.Keys.SET_VALUES] = ([
            ("ros2_context.inputs:domain_id", 89),
            ("imu_publisher.inputs:frameId", "imu"),
            ("imu_publisher.inputs:nodeNamespace",  f"/{robot.name}"),
            ("imu_publisher.inputs:topicName", "imu_raw"),
        ])
    
    def __create_ros_control_graph(self, robot):
        self.__init_controller()
        graph_name = f"{self.root_prim}/{robot.name}/ros_control_action_graph"
        self.gc.edit(
            {"graph_path": graph_name, "evaluator_name": "execution"},
        )
        self.__create_ros_control_nodes(robot=robot)
        self.gc.edit(edit_commands=self.graph_dict)
        set_target_prims(primPath=f"{graph_name}/ros2_publish_joint_states", targetPrimPaths=[robot.prim_path])
        set_target_prims(primPath=f"{graph_name}/isaac_articulation_controller", targetPrimPaths=[robot.prim_path])
        prims.set_targets(
            prim=stage.get_current_stage().GetPrimAtPath(f"{graph_name}/ros2_publish_tf"),
            attribute="inputs:targetPrims",
            target_prim_paths=[robot.prim_path],
        )
  
    def __create_cameras_graph(self, robot, cameras):
        self.__init_controller()
        cam_prim_paths = list()
        graph_name = f"{self.root_prim}/{robot.name}/cameras_action_graph"
        self.gc.edit(
            {"graph_path": graph_name, "evaluator_name": "execution"},
        )
        self.__create_cameras_nodes(robot=robot, cameras=cameras)
        self.gc.edit(edit_commands=self.graph_dict)
        #TODO: move this in a separate method
        for cam_name, cam_conf in cameras.items():
            set_cam_prim_path = f"{graph_name}/isaac_set_{cam_name}"
            cam_prim_path = robot.prim_path + cam_conf.get("prim_path")
            cam_prim_paths.append(cam_prim_path)
            # set cameras to viewports
            prims.set_targets(
                prim=stage.get_current_stage().GetPrimAtPath(set_cam_prim_path),
                attribute="inputs:cameraPrim",
                target_prim_paths=[cam_prim_path],
            )
    
    def __create_lidar_graph(self, robot, lidar):
        self.__init_controller()
        graph_name = f"{self.root_prim}/{robot.name}/lidar_action_graph"
        self.gc.edit(
            {"graph_path": graph_name, "evaluator_name": "execution"},
        )
        self.__create_lidar_nodes(robot, lidar)
        self.gc.edit(edit_commands=self.graph_dict)
        lidar_prim_path = robot.prim_path + lidar.get("prim_path")
        tmp = ["laser_scan_reader", "pcl_reader"]
        for i in tmp:
            prim_path = f"{graph_name}/{i}"
            prims.set_targets(
                    prim=stage.get_current_stage().GetPrimAtPath(prim_path),
                    attribute="inputs:lidarPrim",
                    target_prim_paths=[lidar_prim_path],
                )
    
    def __create_imu_graph(self, robot, imu):
        self.__init_controller()
        graph_name = f"{self.root_prim}/{robot.name}/imu_action_graph"
        self.gc.edit(
            {"graph_path": graph_name, "evaluator_name": "execution"},
        )
        self.__create_imu_nodes(robot, imu)
        self.gc.edit(edit_commands=self.graph_dict)
        imu_prim_path = robot.prim_path + imu.get("prim_path")
        prims.set_targets(
            prim=stage.get_current_stage().GetPrimAtPath(f"{graph_name}/imu_reader"),
            attribute="inputs:imuPrim",
            target_prim_paths=[imu_prim_path],
        )

    def __create_omnigraph(self):
        for r in self.get_robot_assets():
            self.__create_ros_control_graph(robot=r.robot_object)
            if r.cameras:
                self.__create_cameras_graph(robot=r.robot_object, cameras=r.cameras)
            if r.lidar:
                self.__create_lidar_graph(robot=r.robot_object, lidar=r.lidar)
            if r.imu:
                self.__create_imu_graph(robot=r.robot_object, imu=r.imu)

    def spawn_robots(self, robot_config: dict):
        super().spawn_robots(robot_config)
        self.__create_omnigraph()
    
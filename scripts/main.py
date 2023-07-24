from omni.isaac.kit import SimulationApp
import argparse
import yaml

parser = argparse.ArgumentParser(description="Launch IsaacSim and spawn robots")
parser.add_argument(
    "--config-file",
    action="store",
    help='Config file relative path',
    default='../config/jackal_vigna.yaml',  
)
args, unknown = parser.parse_known_args()

with open(args.config_file, "r") as file:
    config_file = yaml.safe_load(file)
    stage_config = config_file.get("stage")
    robot_config = config_file.get("robots")

sim_app = SimulationApp()

from isaac_sim_ros import IsaacRos

def main():
    isaac_ros_simulation = IsaacRos(sim_app, stage_config.get("physics_update_rate"))
    isaac_ros_simulation.add_default_ground_plane()
    isaac_ros_simulation.add_usd_to_stage(stage_config.get("usd_path"), stage_config.get("root_prim_path"))
    isaac_ros_simulation.spawn_robots(robot_config)
    isaac_ros_simulation.start()
    while sim_app._app.is_running() and not sim_app.is_exiting():
        sim_app.update()
    sim_app.close()

if __name__ == "__main__":
    main()

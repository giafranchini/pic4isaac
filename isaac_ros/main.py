"""
TODO:
    - pass paths from cmd line
    - clean IsaacRos.graph_dict after graph creation? 
"""
from omni.isaac.kit import SimulationApp
sim_app = SimulationApp()

from isaac_sim_ros import IsaacRos

STAGE_PATH = "/root/test_usd/pergola/vigna_a_pergola_default.usd"
STAGE_ROOT_PRIM_PATH = "/Empty"
ROBOT_CONFIG_PATH = "/isaac-sim/standalone_examples/isaac_ros/config/jackal_isaac.yaml"

def main():
    isaac_ros_simulation = IsaacRos(sim_app)
    isaac_ros_simulation.add_default_ground_plane()
    # isaac_ros_simulation.add_usd_to_stage(STAGE_PATH, STAGE_ROOT_PRIM_PATH)
    isaac_ros_simulation.spawn_robots(ROBOT_CONFIG_PATH)
    isaac_ros_simulation.start()

    while sim_app._app.is_running() and not sim_app.is_exiting():
        sim_app.update()
    sim_app.close()

if __name__ == "__main__":
    main()

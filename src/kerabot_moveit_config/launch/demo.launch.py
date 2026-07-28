from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "Robot_to_URDF_New_Pakka",
            package_name="kerabot_moveit_config",
        )
        .planning_pipelines(
            # Pilz PTP+Ruckig is the default pipeline used by all our Python scripts.
            # OMPL is registered as a second pipeline so you can request it
            # explicitly via planning_pipeline="ompl" or pipeline_id="ompl".
            default_planning_pipeline="pilz_industrial_motion_planner",
            pipelines=["ompl", "pilz_industrial_motion_planner"],
        )
        .to_moveit_configs()
    )
    return generate_demo_launch(moveit_config)

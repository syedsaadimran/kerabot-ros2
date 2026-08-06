from moveit_configs_utils import MoveItConfigsBuilder

cfg = (
    MoveItConfigsBuilder("Robot_to_URDF_New_Pakka", package_name="kerabot_moveit_config")
    .planning_pipelines(
        default_planning_pipeline="pilz_industrial_motion_planner",
        pipelines=["ompl", "pilz_industrial_motion_planner"],
    )
    .to_moveit_configs()
)
pp = cfg.planning_pipelines
print("=== planning_pipelines keys:", list(pp.keys()))
print()
ompl_plugin  = pp.get("ompl", {}).get("planning_plugin", "MISSING")
pilz_plugin  = pp.get("pilz_industrial_motion_planner", {}).get("planning_plugin", "MISSING")
default_pipe = pp.get("default_planning_pipeline")
pipe_list    = pp.get("planning_pipelines")

print("ompl.planning_plugin  :", ompl_plugin)
print("pilz.planning_plugin  :", pilz_plugin)
print("default_planning_pipeline:", default_pipe)
print("planning_pipelines list  :", pipe_list)
print()

# Check correctness
ok = True
if ompl_plugin != "ompl_interface/OMPLPlanner":
    print("FAIL: ompl pipeline_plugin is wrong:", ompl_plugin)
    ok = False
else:
    print("PASS: ompl pipeline_plugin is correct")

if pilz_plugin != "pilz_industrial_motion_planner/CommandPlanner":
    print("FAIL: pilz pipeline_plugin is wrong:", pilz_plugin)
    ok = False
else:
    print("PASS: pilz pipeline_plugin is correct")

if default_pipe != "pilz_industrial_motion_planner":
    print("FAIL: default_planning_pipeline is wrong:", default_pipe)
    ok = False
else:
    print("PASS: default_planning_pipeline is correct")

print()
print("Overall:", "ALL PASS" if ok else "FAILURES FOUND")

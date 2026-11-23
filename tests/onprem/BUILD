load("@intrinsic-core//intrinsic/assets/build_defs:solution.bzl", "intrinsic_solution")
load("@rules_shell//shell:sh_test.bzl", "sh_test")
load("//bazel:onprem_e2e_test.bzl", "onprem_e2e_test")

# Small simulated solution to check that a solution built in OMTS (including a
# skill built from source) can be deployed and comes up on a leased VM.
intrinsic_solution(
    name = "smoke_solution",
    assets = [
        "@intrinsic-core//intrinsic/simulation/gazebo/asset:gazebo_simulator_type",
        "//models/raw_stock_2x3x5",
        "//src/skills/antipodal_mesh_plan_grasp:antipodal_mesh_plan_grasp_skill",
    ],
    default_operation_mode = "sim",
    instances = [
        "//:gazebo_simulator",
        "//:raw_stock_2x3x5",
    ],
)

# Deploys the solution to the cluster given by $INTRINSIC_ORG and $CLUSTER_NAME
# and waits for it to report ready. Only meaningful inside onprem_e2e_test().
sh_test(
    name = "deploy_smoke_solution",
    srcs = ["deploy_and_wait.sh"],
    data = [":smoke_solution"],
    env = {"SOLUTION": "$(rlocationpath :smoke_solution)"},
    tags = ["manual"],
    use_bash_launcher = True,
)

onprem_e2e_test(
    name = "smoke_solution_test",
    test = ":deploy_smoke_solution",
    timeout_minutes = 20,
)

from glob import glob

from setuptools import setup

package_name = "intent_hybrid_planner"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.json")),
        ("share/" + package_name + "/sdf", glob("sdf/*.sdf")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="muxi",
    maintainer_email="muxi@example.com",
    description="Single unified ROS 2 node for intent-biased hybrid planning.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "intent_hybrid_planner_node = intent_hybrid_planner.intent_hybrid_planner_node:main",
            "spawn_obstacles = intent_hybrid_planner.spawn_obstacles:main",
            "fmp_via_test_node = intent_hybrid_planner.fmp_via_test_node:main",
            "fmp_via_validation = intent_hybrid_planner.fmp_via_validation:main",
            "plot_planning_vis_snapshot = intent_hybrid_planner.plot_planning_vis_snapshot:main",
            "ee_trace_marker = intent_hybrid_planner.ee_trace_marker_node:main",
            "offline_debug_recorder = intent_hybrid_planner.offline_debug_recorder:main",
        ],
    },
)

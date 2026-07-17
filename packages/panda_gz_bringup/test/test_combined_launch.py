import ast
import importlib.util
import os
from pathlib import Path

import pytest


ROOT = Path(__file__).parents[1]
COMBINED = ROOT / "launch" / "panda_moveit_gz.launch.py"
STANDALONE = ROOT / "launch" / "panda_gz.launch.py"


def test_combined_launch_static_generation_contract():
    source = COMBINED.read_text()
    tree = ast.parse(source)
    function_names = {
        node.name for node in tree.body if isinstance(node, ast.FunctionDef)
    }
    assert "generate_launch_description" in function_names
    assert "_launch_setup" in function_names

    for contract in (
        '"hardware_type": "gz"',
        '"controllers_file": str(controllers)',
        '"assisted_grasp":',
        '"panda_gz.launch.py"',
        'package="moveit_ros_move_group"',
        'executable="moveit_bridge_node"',
        '"demo.launch.py"',
        '"backend": backend',
        '"scene": scene',
        '"seed": seed',
        '"grasp_mode": grasp_mode',
        'executable="stack_ready_gate"',
        "OnProcessExit(",
        "target_action=ready_gate",
        "if event.returncode == 0",
        "Shutdown(reason=reason)",
    ):
        assert contract in source

    # These are owned only by the included Gazebo stack. The MoveIt wrapper
    # must not create a second publisher or standalone controller manager.
    assert 'package="robot_state_publisher"' not in source
    assert 'executable="ros2_control_node"' not in source

    standalone = STANDALONE.read_text()
    assert standalone.count('package="robot_state_publisher"') == 1
    assert 'executable="ros2_control_node"' not in standalone
    assert "TimerAction" not in source
    assert "TimerAction" not in standalone
    assert "if event.returncode == 0:" in source
    assert "Shutdown(reason=reason)" in source
    assert "on_exit=[application]" not in source
    assert "gz_args = [" not in standalone
    assert "gz_args = f\"" in standalone


def test_combined_launch_description_generates_when_ros_is_available(monkeypatch):
    pytest.importorskip("launch")
    pytest.importorskip("ament_index_python")
    pytest.importorskip("moveit_configs_utils")
    monkeypatch.setenv("ROS_LOG_DIR", os.path.join("/tmp", "panda_gz_launch_test"))
    spec = importlib.util.spec_from_file_location("panda_moveit_gz_launch", COMBINED)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    argument_names = {
        entity.name for entity in description.entities if hasattr(entity, "name")
    }
    assert {
        "backend",
        "scene",
        "seed",
        "grasp_mode",
        "headless",
        "use_rviz",
        "start_application",
        "output_path",
        "physics_model_validated",
    } <= argument_names

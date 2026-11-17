"""Writer planner fallback control and preservation of native Nav2 recoveries."""
from copy import deepcopy
from pathlib import Path
import shutil
import subprocess
from types import SimpleNamespace
from unittest.mock import Mock
import xml.etree.ElementTree as ET

import pytest
import yaml

pytest.importorskip("ament_index_python")
from ament_index_python.packages import get_package_share_directory

ROOT = Path(__file__).resolve().parents[1]
TREE = ROOT / "src/living_map_writer/behavior_trees/navigate_with_planner_fallback.xml"


def planner_fallback(root):
    candidates = [n for n in root.iter("Fallback")
                  if len(n.findall(".//ComputePathToPose")) == 2]
    assert len(candidates) == 1
    return candidates[0]


def normalized(node):
    return node.tag, sorted(node.attrib.items()), [normalized(c) for c in node]


def test_only_planning_leaf_changes_from_installed_nav2_recovery_tree():
    stock = Path(get_package_share_directory("nav2_bt_navigator")) / "behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml"
    root = ET.parse(TREE).getroot()
    fallback = planner_fallback(root)
    for parent in root.iter():
        for index, child in enumerate(parent):
            if child is fallback:
                parent[index] = deepcopy(fallback.find("ComputePathToPose"))
                break
    assert normalized(root) == normalized(ET.parse(stock).getroot())


def test_dijkstra_keeps_primary_costmap_policy_and_only_handles_no_valid_path():
    pytest.importorskip("nav2_msgs.action")
    from nav2_msgs.action import ComputePathToPose
    fallback = planner_fallback(ET.parse(TREE).getroot())
    guard = fallback.find("Sequence/AreErrorCodesPresent")
    assert guard.attrib["error_codes_to_check"] == str(ComputePathToPose.Result.NO_VALID_PATH)
    assert guard.attrib["error_code"] == "{compute_path_error_code}"
    config = yaml.safe_load((ROOT / "config/nav2/writer.yaml").read_text())
    planner = config["/writer/planner_server"]["ros__parameters"]
    secondary_id = fallback.find("Sequence/ComputePathToPose").attrib["planner_id"]
    assert planner["planner_plugins"] == ["GridBased", secondary_id]
    assert planner["GridBased"]["use_astar"] is True
    assert planner[secondary_id] == {**planner["GridBased"], "use_astar": False}


@pytest.mark.parametrize("kind", ["frontier", "home"])
def test_safe_writer_goals_use_packaged_tree_and_unsafe_goals_are_still_rejected(kind):
    pytest.importorskip("rclpy")
    from builtin_interfaces.msg import Time
    from living_map_writer.frontier_node import FrontierExplorer
    e = SimpleNamespace(minimum_goal_x=-.2616, behavior_tree=str(TREE), nav=Mock(),
                        _goal_sent=Mock(), _goal_failed=Mock(), get_logger=Mock(return_value=Mock()),
                        watchdog=Mock(), _now_s=lambda: 0., _on_feedback=Mock(),
                        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time)))
    FrontierExplorer._send_goal(e, 2., 1., kind)
    goal = e.nav.send_goal_async.call_args.args[0]
    assert goal.behavior_tree == str(TREE)
    assert goal.pose.header.frame_id == "map"
    assert goal.pose.pose.position.x == 2. and goal.pose.pose.position.y == 1.
    e.nav.send_goal_async.reset_mock()
    FrontierExplorer._send_goal(e, -.724, 0., kind)
    e.nav.send_goal_async.assert_not_called()
    e._goal_failed.assert_called_once_with(-.724, 0., kind, "entrance_rejected")


def test_actual_production_fallback_with_native_behavior_tree_cpp(tmp_path):
    """Native control flow only; actual planner/collision tests remain separate."""
    compiler = shutil.which("g++")
    if compiler is None:
        pytest.skip("Native compiler unavailable; no dependency installation attempted")
    prefix = Path(get_package_share_directory("rclcpp")).parents[1]
    include = prefix / "include"
    library = prefix / "lib"
    source = ROOT / "tests/native/verify_planner_fallback.cpp"
    binary = tmp_path / "verify_planner_fallback"
    cmd = [compiler, "-std=c++17", "-O1", str(source), "-I" + str(include)]
    cmd += ["-I" + str(p) for p in sorted(include.iterdir()) if p.is_dir()]
    cmd += ["-L" + str(library), "-lbehaviortree_cpp", "-lrclcpp",
            "-Wl,-rpath," + str(library), "-o", str(binary)]
    built = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
    assert built.returncode == 0, built.stdout + built.stderr
    root = ET.Element("root", BTCPP_format="4", main_tree_to_execute="MainTree")
    tree = ET.SubElement(root, "BehaviorTree", ID="MainTree")
    tree.append(deepcopy(planner_fallback(ET.parse(TREE).getroot())))
    extracted = tmp_path / "actual-production-fallback.xml"
    ET.ElementTree(root).write(extracted, encoding="unicode")
    run = subprocess.run([str(binary), str(extracted)], capture_output=True, text=True, timeout=15)
    assert run.returncode == 0, run.stdout + run.stderr
    assert "11 native BehaviorTree.CPP control-flow cases passed" in run.stdout

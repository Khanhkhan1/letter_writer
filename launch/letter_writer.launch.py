#!/usr/bin/env python3
"""Launch UR3/UR3e Gazebo simulation + MoveIt 2 + the letter_writer control node.

Reuses ur_simulation_gz's ur_sim_control.launch.py (Gazebo sim + ros2_control
controllers) and ur_moveit_config's ur_moveit.launch.py (MoveIt move_group +
RViz) unmodified -- the same two launch files that ur_simulation_gz's own
ur_sim_moveit.launch.py composes -- included here so that `gazebo_gui` can be
forwarded. This starts Gazebo headless (`gz sim -s`): on macOS, gz-sim's
combined server+GUI process is not supported (gazebosim/gz-sim#44), so
Gazebo runs headless and RViz (MoveIt's "Show Trail" option) is used to
observe the end-effector path instead. Set gazebo_gui:=true on Ubuntu/Linux
to get the normal Gazebo window.

ur_moveit_config's ur_moveit.launch.py is started as a separate `ros2
launch` OS process (ExecuteProcess) rather than IncludeLaunchDescription in
the same process. On the RoboStack/macOS setup this package was developed
on, running its internal `xacro` Command substitution in the same Python
process/event loop as ur_control's already-running async subprocesses
(gz sim, controller spawners, ...) intermittently segfaults xacro
(a fork()-safety interaction between CPython's subprocess handling and
`launch`'s asyncio event loop). A separate process sidesteps it entirely.
This is harmless overhead on Linux.

Then starts the student's letter_writer_node after a delay so the
controllers and MoveIt are fully up before planning begins.
"""

import glob
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _macos_python_preload_env():
    """RoboStack/macOS workaround (see module docstring / README).

    letter_writer_node links moveit_ros_planning_interface, which pulls in
    message packages whose rosidl_generator_py (Python extension) libraries
    get linked in too and are built with "-undefined dynamic_lookup" --
    only resolvable if libpython is already loaded in-process. Without this,
    the node dies at startup with:
      dyld: symbol not found in flat namespace '__Py_TrueStruct'
    Scoped to just this node's env (not the whole launch tree). No-op on
    Linux, where this isn't needed.
    """
    if sys.platform != "darwin" or "CONDA_PREFIX" not in os.environ:
        return {}
    candidates = glob.glob(os.path.join(os.environ["CONDA_PREFIX"], "lib", "libpython3.*.dylib"))
    return {"DYLD_INSERT_LIBRARIES": candidates[0]} if candidates else {}


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    startup_delay = LaunchConfiguration("startup_delay")

    declared_arguments = [
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur3",
            description="Type of UR robot to simulate (ur3, ur3e, ...).",
        ),
        DeclareLaunchArgument(
            "gazebo_gui",
            default_value="false",
            description="Start Gazebo with its own GUI window. Must be false on "
            "macOS (gz-sim combined server+GUI is unsupported there); use RViz's "
            "MotionPlanning 'Show Trail' to see the end-effector path instead.",
        ),
        DeclareLaunchArgument(
            "startup_delay",
            default_value="15.0",
            description="Seconds to wait for Gazebo/controllers/MoveIt to come up "
            "before starting letter_writer_node.",
        ),
    ]

    ur_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_simulation_gz"), "launch", "ur_sim_control.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            "gazebo_gui": gazebo_gui,
            "launch_rviz": "false",
            "controllers_file": PathJoinSubstitution(
                [FindPackageShare("letter_writer"), "config", "ur_controllers_relaxed.yaml"]
            ),
        }.items(),
    )

    ur_moveit = TimerAction(
        # Small delay so ur_control's own controller_manager/spawners are up
        # before MoveIt tries to talk to them; see module docstring for why
        # this runs as a separate `ros2 launch` process.
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "launch",
                    "ur_moveit_config",
                    "ur_moveit.launch.py",
                    ["ur_type:=", ur_type],
                    "use_sim_time:=true",
                    "launch_rviz:=false",
                ],
                output="screen",
                name="ur_moveit_launch",
            )
        ],
    )

    # Our own RViz, started separately (rather than via ur_moveit's
    # launch_rviz:=true) so we can load a custom config with a "Letter
    # Trace" Marker display pre-added, subscribed to letter_writer_node's
    # /letter_trace topic -- otherwise the drawn letter's shape has no
    # persistent visual trace (MoveIt's own "Show Trail" only shows a
    # preview of a *planned* path, not the accumulated actually-drawn
    # letter). Gazebo has no usable GUI on macOS (see module docstring),
    # so this is the only live view of the letter being drawn there.
    rviz = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_letter_writer",
                output="log",
                arguments=[
                    "-d",
                    PathJoinSubstitution(
                        [FindPackageShare("letter_writer"), "config", "letter_writer.rviz"]
                    ),
                ],
                parameters=[{"use_sim_time": True}],
            )
        ],
    )

    letter_writer_node = Node(
        package="letter_writer",
        executable="letter_writer_node",
        name="letter_writer_node",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("letter_writer"), "config", "letter_params.yaml"]
            ),
            {"use_sim_time": True},
        ],
        additional_env=_macos_python_preload_env(),
    )

    delayed_letter_writer = TimerAction(period=startup_delay, actions=[letter_writer_node])

    return LaunchDescription(
        declared_arguments + [ur_control, ur_moveit, rviz, delayed_letter_writer]
    )

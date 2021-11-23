# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
from moveit_msgs.msg import PlanningSceneWorld
import geometry_msgs.msg

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

# https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html#getting-started


moveit_commander.roscpp_initialize(sys.argv + ['__ns:=/hero'])
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
group_name = "arm"

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface(ns='/hero', synchronous=True)
move_group = moveit_commander.MoveGroupCommander(group_name)

# print("============ Reference frame: %s" % move_group.get_planning_frame())
# print("============ Reference frame: %s" % move_group.get_end_effector_link())
# print("============ Robot Groups:")
# print(robot.get_group_names())
# print("============ Printing robot state")
# print(robot.get_current_state())
# print("============")
# print("============ Generating plan 1")


# # # Plan to a joint goal
# joint_goal = move_group.get_current_joint_values()
# # # random position [0.3532629789664938, -0.4293987981401086, 1.1836900769463248, -0.840112834258244, 2.5121319061568474, 0.0]
# # # default position [3.8312383143695795e-05, 9.425236700710826e-05, 0.0004889303350981145, -1.5727771742891148, -0.0005135843578116805, 0.0]
# # # breakpoint()
# joint_goal[0] = 0.3532629789664938
# joint_goal[1] = -0.4293987981401086
# joint_goal[2] = 1.1836900769463248
# joint_goal[3] = -0.840112834258244
# joint_goal[4] = 2.5121319061568474
# joint_goal[5] = 0.0
# # The go command can be called with joint values, poses, or without any # parameters if you have already set the pose or joint target for the group
# move_group.go(joint_goal, wait=True)
# # Calling ``stop()`` ensures that there is no residual movement
# move_group.stop()





# Plan to a pose goal
# random position
#     x: -0.3836945289064705
#     y: -0.1897794942517491
#     z: 1.0836714600788058
#   orientation:
#     x: 0.36501739322064874
#     y: 0.35051700584642304
#     z: -0.7151738979603429
#     w: 0.48210624029882654
# home position
#     x: -0.3285649572665905
#     y: -0.11476536655772912
#     z: 0.6727896702977942
#   orientation:
#     x: 0.19983473862484283
#     y: -0.679034617556007
#     z: 0.1990716511305686
#     w: 0.6777525677675129

# # https://answers.ros.org/question/255647/orientation-constraint-for-link-tip-is-probably-incorrect-warning/
# pose_goal = geometry_msgs.msg.Pose()
# # pose_goal = move_group.get_current_pose(end_effector_link=move_group.get_end_effector_link())
# # normalized home position orientation
# # still not working, not really an idea why.....
# pose_goal.orientation.x = 0.284738041
# pose_goal.orientation.y = -0.6644
# pose_goal.orientation.z = 0.189825361
# pose_goal.orientation.w = 0.6644
# pose_goal.position.x = -0.3
# pose_goal.position.y = -0.1
# pose_goal.position.z = 0.7
#
#
# move_group.set_pose_target(pose_goal)
# plan = move_group.go(wait=True)
# move_group.stop()
# move_group.clear_pose_targets()



## Plan carthesian path (based on paths, does not work....)
# waypoints = []
# scale = 1.0
# wpose = move_group.get_current_pose().pose
# wpose.position.z -= scale * 0.1  # First move up (z)
# wpose.position.y += scale * 0.2  # and sideways (y)
# waypoints.append(copy.deepcopy(wpose))
#
# wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
# waypoints.append(copy.deepcopy(wpose))
#
# wpose.position.y -= scale * 0.1  # Third move sideways (y)
# waypoints.append(copy.deepcopy(wpose))
#
# # We want the Cartesian path to be interpolated at a resolution of 1 cm
# # which is why we will specify 0.01 as the eef_step in Cartesian
# # translation.  We will disable the jump threshold by setting it to 0.0,
# # ignoring the check for infeasible jumps in joint space, which is sufficient
# # for this tutorial.
# (plan, fraction) = move_group.compute_cartesian_path(
#     waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
# )  # jump_threshold
#
# move_group.execute(plan, wait=True)



# Add object
# no objects seem to be added.....
box_pose = geometry_msgs.msg.PoseStamped()
box_pose.header.frame_id = move_group.get_end_effector_link()
box_pose.pose.orientation.w = 1.0
box_pose.pose.position.z = 0.11  # above the panda_hand frame
box_name = "box"
scene.add_box(box_name, box_pose, size=(0.075, 0.075, 0.075))


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "It works")
    foo = data.collision_objects
    breakpoint()

# rospy.wait_for_service('/hero/ed/moveit_scene')
# add_two_ints = rospy.
rospy.Subscriber("/hero/planning_scene_world", PlanningSceneWorld, callback)
rospy.spin()





# touch_links = robot.get_link_names(group="gripper")
# scene.attach_box(move_group.get_end_effector_link(), box_name, touch_links=touch_links)

# scene.remove_attached_object(move_group.get_end_effector_link(), name=box_name)
#
# scene.remove_world_object(box_name)

# implement usage of ed moveit
# rosservice call hero/ed/moveit should publish the collision env to a specific topic
# ed moveit installed, but hero_bringup in feature/moveit does not work, grasping_tmc (as in ed_moveit) dus not exist
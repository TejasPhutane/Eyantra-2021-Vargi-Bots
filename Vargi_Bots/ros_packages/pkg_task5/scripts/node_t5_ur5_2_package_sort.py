#!/usr/bin/env python

''' This node is used for controlling the ur5_2 robot, sort packages into its respctive coloured bins. '''

import sys
import copy
import math
from threading import Thread
from datetime import datetime, date, timedelta
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
import actionlib
import tf2_ros
import tf2_msgs.msg

from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import vacuumGripperRequest
from pkg_vb_sim.srv import vacuumGripperResponse
from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.srv import conveyorBeltPowerMsgRequest
from pkg_vb_sim.srv import conveyorBeltPowerMsgResponse
from pkg_task4.srv import camera_packages
from pkg_task4.srv import camera_packagesResponse
from pkg_vb_sim.msg import LogicalCameraImage

from pkg_ros_iot_bridge.msg import msgRosIotAction
# Message Class that is used by ROS Actions internally
from pkg_ros_iot_bridge.msg import msgRosIotGoal
# Message Class that is used for Action Goal Messages
from pkg_ros_iot_bridge.msg import msgRosIotResult
# Message Class that is used for Action Result Messages
from pkg_ros_iot_bridge.msg import msgRosIotFeedback
# Message Class that is used for Action Feedback Messages
from pkg_ros_iot_bridge.msg import msgMqttSub
# Message Class for MQTT Subscription Messages

''' Class to initialize the sort process. '''
class Ur5PackageSort:

    # Constructor
    def __init__(self):
        
        # Initialize ROS Node
        rospy.init_node('node_t4_ur5_2_package_sort', anonymous=True) 
        rospy.sleep(10)
       
        # Wait for service
        rospy.wait_for_service('/2Dcamera_packages_type') 

        # Subscribe to logical camera to get package name and position
        rospy.Subscriber("/eyrc/vb/logical_camera_2", LogicalCameraImage, self.logical_camera_callback)
        rospy.Subscriber("/Orders_to_ship", String, self.orders_callback)

        # Load variables for moveit!
        self._robot_ns = '/'  + "ur5_2"
        self._planning_group = "manipulator"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description=self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description=self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher(self._robot_ns + '/move_group/display_planned_path',
                                                             moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient(self._robot_ns + '/execute_trajectory',
                                                                        moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()
        self._curr_state = self._robot.get_current_state()    

        rospy.set_param('/ur5_2_vacuum_gripper_service', False)
        rospy.set_param('/ur5_1_vacuum_gripper_service', False)
        rospy.set_param('/conveyor_belt_service', False)

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._group.set_planning_time(99)
        self._conveyor_speed = 75
        self._is_arm_moving = False
        self._current_package = ""
        self._box_name = ""
        self.orders_list = []
        #Initialize tf2
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)

        rospy.loginfo('\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo('\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo('\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')
        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

        ## MQTT Client

        # Initialize Action Client
        self._ac = actionlib.ActionClient('/action_ros_iot',
                                          msgRosIotAction)

        param_config_iot = rospy.get_param('config_pyiot')

        # Store the ROS Topic to get the Orders from bridge action server
        self._config_mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']

        # Dictionary to Store all the goal handels
        self._goal_handles = {}

        # Wait for Action Server that will use the action - '/action_iot_ros' to start
        self._ac.wait_for_server()
        rospy.loginfo("Action server up, we can send goals.")

    ''' Get the name and postion of the package. '''
    def logical_camera_callback(self, package_name):
        if  package_name.models == []:
            self._box_name = "empty"
            self._box_distance = 0
        else:
            pkg = package_name.models[-1]
            self._box_name = pkg.type
            self._box_distance_y = pkg.pose.position.y

    ''' Orders to be shipped '''
    def orders_callback(self, msg):
        rospy.loginfo('Order to ship : ' + msg.data)
        self.orders_list.append(eval(msg.data))

    '''This function will be called when there is a change of
       state in the Action Client State Machine. '''
    def on_transition(self, goal_handle):
    
        # from on_goal() to on_transition(). goal_handle generated by send_goal() is used here.
        result = msgRosIotResult()
        index = 0
        for i in self._goal_handles:
            if self._goal_handles[i] == goal_handle:
                index = i
                break

        rospy.loginfo("Transition Callback. Client Goal Handle #: " + str(index))
        rospy.loginfo("Comm. State: " + str(goal_handle.get_comm_state()))
        rospy.loginfo("Goal Status: " + str(goal_handle.get_goal_status()))

        # Comm State - Monitors the State Machine of the Client which is different from Server's
        # Comm State = 2 -> Active
        # Comm State = 3 -> Wating for Result
        # Comm State = 7 -> Done

        # if (Comm State == ACTIVE)
        if goal_handle.get_comm_state() == 2:
            rospy.loginfo(str(index) + ": Goal just went active.")

        # if (Comm State == DONE)
        if goal_handle.get_comm_state() == 7:
            rospy.loginfo(str(index) + ": Goal is DONE")
            rospy.loginfo(goal_handle.get_terminal_state())

            # get_result() gets the result produced by the Action Server
            result = goal_handle.get_result()
            rospy.loginfo(result.flag_success)

            if result.flag_success == True:
                rospy.loginfo("Goal successfully completed. Client Goal Handle #: " + str(index))
            else:
                rospy.loginfo("Goal failed. Client Goal Handle #: " + str(index))

    ''' This function is used to send Goals to MQTT client. '''
    def send_goal_to_mqtt_client(self, arg_protocol, arg_mode, arg_topic, arg_message):
        # Create a Goal Message object
        goal = msgRosIotGoal()

        goal.protocol = arg_protocol
        goal.mode = arg_mode
        goal.topic = arg_topic
        goal.message = arg_message

        rospy.loginfo("Sending to mqtt client")

        # self.on_transition - It is a function pointer to a function which will be called when
        #                       there is a change of state in the Action Client State Machine
        goal_handle = self._ac.send_goal(goal,
                                         self.on_transition,
                                         None)

        return goal_handle

    ''' Function to get detected packages. '''
    def camera1_callback(self):
        get_packages_type = rospy.ServiceProxy('/2Dcamera_packages_type', camera_packages)
        try:
            self.get_packages = get_packages_type(True)
            self._package_colours = eval(self.get_packages.pack_type)
            return self.get_packages.pack_type
        except rospy.ServiceException as exc:
            print "Service did not process request: " + str(exc)

    ''' Function to move robot at a specific position in it's workspace. '''
    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)
        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move
        self._group.stop()
        self._group.clear_pose_targets()

        if flag_plan == True:
            rospy.loginfo('\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr('\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    ''' Function to set the poses until it reaches max attempt. '''
    def hard_set_go_to_pose(self, arg_pose, arg_max_attempts):

        number_attempts = 0
        flag_success = False

        while number_attempts <= arg_max_attempts and flag_success is False:
            number_attempts += 1
            flag_success = self.go_to_pose(arg_pose)
            rospy.logwarn("attempts: {}".format(number_attempts))

    ''' Set UR5_2 joint angles. '''
    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        self._group.set_joint_value_target(arg_list_joint_angles)
        flag_plan = self._group.go(wait=True)
        list_joint_values = self._group.get_current_joint_values()
        pose_values = self._group.get_current_pose().pose

        if flag_plan == True:
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan


    ''' Function to set the angles until it reaches max attempt. '''
    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):

        number_attempts = 0
        flag_success = False

        while ((number_attempts <= arg_max_attempts) and  (flag_success is False)):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts))

    ''' Function to calculate the path through waypoints. '''
    def ee_cartesian_translation(self, trans_x, trans_y, trans_z):
        # 1. Create a empty list to hold waypoints
        waypoints = []

        # 2. Add Current Pose to the list of waypoints
        waypoints.append(self._group.get_current_pose().pose)

        # 3. Create a New waypoint
        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = waypoints[0].position.x + (trans_x)
        wpose.position.y = waypoints[0].position.y + (trans_y)
        wpose.position.z = waypoints[0].position.z + (trans_z)
        # This to keep EE parallel to Ground Plane
        wpose.orientation.x = -0.5
        wpose.orientation.y = -0.5
        wpose.orientation.z = 0.5
        wpose.orientation.w = 0.5

        # 4. Add the new waypoint to the list of waypoints
        waypoints.append(copy.deepcopy(wpose))

        # 5. Compute Cartesian Path connecting the waypoints in the list of waypoints
        (plan, fraction) = self._group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # Step Size, distance between two adjacent computed waypoints will be 1 cm
            0.0)         # Jump Threshold
        rospy.loginfo("Path computed successfully. Moving the arm.")

        num_pts = len(plan.joint_trajectory.points)
        if (num_pts >= 3):
            del plan.joint_trajectory.points[0]
            del plan.joint_trajectory.points[1]

        # 6. Make the arm follow the Computed Cartesian Path
        self._group.execute(plan)

    ''' Function to attach box to UR5_2 vacuum gripper. '''
    def attach_box(self):
        if rospy.get_param('/ur5_1_vacuum_gripper_service') == True or rospy.get_param('/conveyor_belt_service') == True:
            rospy.loginfo_once("Waiting for Service")
            rospy.sleep(1)
        rospy.set_param('/ur5_2_vacuum_gripper_service', True)
        try:
            rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2')
            self.attach2 = rospy.ServiceProxy('eyrc/vb/ur5/activate_vacuum_gripper/ur5_2',
                                              vacuumGripper)
            self.attach2(True)
            rospy.set_param('/ur5_2_vacuum_gripper_service', False)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            print "Trying to reconnect service"
            rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2')
            rospy.set_param('/ur5_2_vacuum_gripper_service', True)
            self.attach2 = rospy.ServiceProxy('eyrc/vb/ur5/activate_vacuum_gripper/ur5_2',
                                              vacuumGripper)
            self.attach2(True)
            rospy.set_param('/ur5_2_vacuum_gripper_service', False)

    ''' Function to detach box from UR5_2 vacuum gripper. '''
    def detach_box(self):
        if rospy.get_param('/ur5_1_vacuum_gripper_service') == True or rospy.get_param('/conveyor_belt_service') == True:
            rospy.loginfo_once("Waiting for Service")
            rospy.sleep(1)
        rospy.set_param('ur5_2_vacuum_gripper_service', True)
        try:
            rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2')
            self.attach2 = rospy.ServiceProxy('eyrc/vb/ur5/activate_vacuum_gripper/ur5_2',
                                              vacuumGripper)
            self.attach2(False)
            rospy.set_param('ur5_2_vacuum_gripper_service', False)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            print "Trying to reconnect service"
            rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2')
            self.attach2 = rospy.ServiceProxy('eyrc/vb/ur5/activate_vacuum_gripper/ur5_2',
                                              vacuumGripper)
            self.attach2(False)
            rospy.set_param('ur5_2_vacuum_gripper_service', False)

    ''' Conveyor belt power control (0-100). '''
    def conveyer_belt_control(self, power):
        if rospy.get_param('/ur5_1_vacuum_gripper_service') == True and rospy.get_param('/ur5_2_vacuum_gripper_service') == True:
            rospy.loginfo_once("Waiting for Service")
            rospy.sleep(1)
        rospy.set_param('/conveyor_belt_service', True)
        try:
            rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
            conveyer_power = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
            conveyer_power(power)
            rospy.set_param('/conveyor_belt_service', False)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            print "Trying to reconnect service"
            rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
            rospy.set_param('/conveyor_belt_service', True)
            conveyer_power = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
            conveyer_power(power)
            rospy.set_param('conveyor_belt_service', False)


    ''' UR5_2 robot arm pick and place routine. '''
    def robot_pick_place(self, _current_package, bin_name, home_pose, ordersheet):
        rospy.sleep(0.4)
        self.attach_box()
        self.ee_cartesian_translation(0, 0, 0.19)
        self._is_arm_moving = True
        self.hard_set_go_to_pose(bin_name, 5)
        self.detach_box()

        if ordersheet['Priority'] == 'HP':
            est_dt = str(date.today() + timedelta(days=1))
        elif ordersheet['Priority'] == 'MP':
            est_dt = str(date.today() + timedelta(days=3))
        elif ordersheet['Priority'] == 'LP':
            est_dt = str(date.today() + timedelta(days=5))

        dt_string = datetime.now().strftime("%d/%m/%Y %H:%M:%S") 
    
        info = {'id':'OrdersShipped', 'Team Id':'VB#693', 'Unique Id':'RRCneYRC',
                'Order ID':ordersheet['Order ID'], 'City':ordersheet['City'],
                'Item':ordersheet['Item'], 'Priority':ordersheet['Priority'],
                'Shipped Quantity':'1', 'Cost':ordersheet['Cost'], 'Shipped Status':'YES',
                'Shipped Date and Time': dt_string, 'Estimated Time of Delivery':est_dt}
        message = str(info)
        goal_handle = self.send_goal_to_mqtt_client("spreadsheet", "pub",
                                                    self._config_mqtt_pub_topic, message)
        self._goal_handles['Order Shipped'] = goal_handle

        self.hard_set_go_to_pose(home_pose, 5)
        self._is_arm_moving = False

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

''' Main Function. '''
def main():

    # Wait for Initializing Gazebo and Rviz

    # Create UR5 object
    ur5_2 = Ur5PackageSort()

    pkg_names=ur5_2.camera1_callback()
    
    box_length = 0.15               # Length of the Package
    vacuum_gripper_width = 0.115    # Vacuum Gripper Width
    delta = vacuum_gripper_width + (box_length/2)  # 0.19

    ur5_2_home_pose = geometry_msgs.msg.Pose()
    ur5_2_home_pose.position.x = -0.8
    ur5_2_home_pose.position.y = 0
    ur5_2_home_pose.position.z = 1 + delta
    # This to keep EE parallel to Ground Plane
    ur5_2_home_pose.orientation.x = -0.5
    ur5_2_home_pose.orientation.y = -0.5
    ur5_2_home_pose.orientation.z = 0.5
    ur5_2_home_pose.orientation.w = 0.5

    ur5_pose_bin_red = geometry_msgs.msg.Pose()
    ur5_pose_bin_red.position.x = -0.07
    ur5_pose_bin_red.position.y = 0.82
    ur5_pose_bin_red.position.z = 1.273
    ur5_pose_bin_red.orientation.x = -0.5
    ur5_pose_bin_red.orientation.y = -0.5
    ur5_pose_bin_red.orientation.z = 0.5
    ur5_pose_bin_red.orientation.w = 0.5

    ur5_pose_bin_yellow = geometry_msgs.msg.Pose()
    ur5_pose_bin_yellow.position.x = 0.83
    ur5_pose_bin_yellow.position.y = 0.2
    ur5_pose_bin_yellow.position.z = 1.273
    ur5_pose_bin_yellow.orientation.x = -0.5
    ur5_pose_bin_yellow.orientation.y = -0.5
    ur5_pose_bin_yellow.orientation.z = 0.50
    ur5_pose_bin_yellow.orientation.w = 0.50

    ur5_pose_bin_green = geometry_msgs.msg.Pose()
    ur5_pose_bin_green.position.x = -0.1
    ur5_pose_bin_green.position.y = -0.77
    ur5_pose_bin_green.position.z = 1.273
    ur5_pose_bin_green.orientation.x = -0.5
    ur5_pose_bin_green.orientation.y = -0.5
    ur5_pose_bin_green.orientation.z = 0.5
    ur5_pose_bin_green.orientation.w = 0.5

    # Names of packages and their respective bins in gazebo
    packages = {"red":ur5_pose_bin_red, "yellow":ur5_pose_bin_yellow, "green":ur5_pose_bin_green}

    # Start Conveyor belt and go to home pose
    ur5_2.conveyer_belt_control(ur5_2._conveyor_speed)
    ur5_2.go_to_pose(ur5_2_home_pose)
    ur5_2._is_arm_moving == False

    #1.From the given packages,stop any package at centre of camera
    #2.If ithe robot is at home position then change the
    #  detected package name to current package name
    #3.Start robot arm routine function in separate thread and after some time start coneyor belt
    while not rospy.is_shutdown():
        if (ur5_2._box_name in ur5_2._package_colours) and ur5_2._box_distance_y <= 0.065:
            ur5_2.conveyer_belt_control(0)
            rospy.loginfo_once("***** Package found , now sorting ******")
            if ur5_2._is_arm_moving == False:
                ur5_2._current_package = ur5_2._box_name

                # Thread for robot arm routine
                thread_robot_pick_place = Thread(name="pick_place",
                                                 target=ur5_2.robot_pick_place,
                                                 args=(ur5_2._current_package, packages[ur5_2._package_colours[ur5_2._current_package]],
                                                       ur5_2_home_pose, ur5_2.orders_list.pop(-1)))
                thread_robot_pick_place.start()
                rospy.sleep(0.1)
                ur5_2.conveyer_belt_control(ur5_2._conveyor_speed)

                ur5_2._is_arm_moving = True


        else:
            ur5_2.conveyer_belt_control(ur5_2._conveyor_speed)


    del ur5_2
if __name__ == '__main__':
    main()

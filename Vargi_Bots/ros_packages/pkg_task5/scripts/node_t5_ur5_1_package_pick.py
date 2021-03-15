#!/usr/bin/env python

''' This node is used for controlling the ur5_1 arm and conveyor belt. '''

import sys
import math
import datetime

from datetime import datetime

import yaml
import rospy
import rospkg
import moveit_commander
import moveit_msgs.msg
import actionlib

from std_srvs.srv import Empty
from std_msgs.msg import String
from pkg_vb_sim.srv import vacuumGripper
from pkg_task5.srv import camera_packages

from pkg_ros_iot_bridge.msg import msgRosIotAction
# Message Class that is used by ROS Actions internally
from pkg_ros_iot_bridge.msg import msgRosIotGoal
# Message Class that is used for Action Goal Messages
from pkg_ros_iot_bridge.msg import msgRosIotResult
# Message Class that is used for Action Result Messages
from pkg_ros_iot_bridge.msg import msgRosIotFeedback
# # Message Class that is used for Action Feedback Messages
from pkg_ros_iot_bridge.msg import msgMqttSub
# Message Class for MQTT Subscription Messages

''' Class to initiate the pick & place process. '''
class Ur5PickPlace:

    # Constructor
    def __init__(self):

        self._original_orders = []
        self.HigherPriorityOrder = []
        self.ur5_1_home_pose = [math.radians(-90), math.radians(-90), math.radians(0),
                                math.radians(-90), math.radians(-90), math.radians(90)]
        self.ur5_1_conveyor_pose = [math.radians(7.8), math.radians(-139.4), math.radians(-57.6),
                                    math.radians(-72.8), math.radians(89.9), math.radians(7.8)]
        ur5_1_pkg00 = [math.radians(-55.8), math.radians(-67.0), math.radians(1.2),
                       math.radians(-114.1), math.radians(-121.3), math.radians(90)]
        ur5_1_pkg01 = [math.radians(-118.9), math.radians(-85.6), math.radians(18.7),
                       math.radians(-113.1), math.radians(-61.0), math.radians(90.0)]
        ur5_1_pkg02 = [math.radians(55.7), math.radians(-117.0), math.radians(5.4),
                       math.radians(-68.4), math.radians(124.2), math.radians(90)]
        ur5_1_pkg10 = [math.radians(-55.1), math.radians(-96.9), math.radians(82.6),
                       math.radians(-165.7), math.radians(-124.8), math.radians(90)]
        ur5_1_pkg11 = [math.radians(-122.7), math.radians(-116.5), math.radians(95.9),
                       math.radians(-159.3), math.radians(-57.2), math.radians(90.0)]
        ur5_1_pkg12 = [math.radians(54.4), math.radians(-84.5), math.radians(-83.6),
                       math.radians(-9.3), math.radians(126.7), math.radians(90)]
        ur5_1_pkg20 = [math.radians(-55.09), math.radians(-96.44), math.radians(87.31),
                       math.radians(9.035), math.radians(125.49), math.radians(90)]
        ur5_1_pkg21 = [math.radians(116.01), math.radians(-61.96), math.radians(-129.27),
                       math.radians(10.33), math.radians(62.64), math.radians(90)]  
        ur5_1_pkg22 = [math.radians(55.5), math.radians(-85.8), math.radians(-114.2),
                       math.radians(20.8), math.radians(124.7), math.radians(90.0)]
        ur5_1_pkg30 = [math.radians(-55.08), math.radians(-91.64), math.radians(117.76),
                       math.radians(-26.22), math.radians(125.48), math.radians(90.0)]
        ur5_1_pkg31 = [math.radians(-121.6), math.radians(-115.9), math.radians(135.1),
                       math.radians(-19.2), math.radians(58.3), math.radians(90)]
        ur5_1_pkg32 = [math.radians(-160.73), math.radians(-92.61), math.radians(118.27),
                       math.radians(-25.89), math.radians(19.84), math.radians(90)]

        # Names of packages and their respective bins in gazebo
        self.packages_name_position = {"packagen00":ur5_1_pkg00, "packagen01":ur5_1_pkg01,
                                       "packagen02":ur5_1_pkg02, "packagen10":ur5_1_pkg10,
                                       "packagen11":ur5_1_pkg11, "packagen12":ur5_1_pkg12,
                                       "packagen20":ur5_1_pkg20, "packagen21":ur5_1_pkg21,
                                       "packagen22":ur5_1_pkg22, "packagen30":ur5_1_pkg30,
                                       "packagen31":ur5_1_pkg31, "packagen32":ur5_1_pkg32}


        # Initialize ROS Node
        rospy.init_node('node_t5_ur5_1_package_pick', anonymous=True)
        rospy.sleep(15)
        self.publish_orders = rospy.Publisher('/Orders_to_ship', String, queue_size=10)

        # Wait for service
        rospy.wait_for_service('/2Dcamera_packages_type')
        
        # Load variables for moveit!
        self._robot_ns = '/'  + "ur5_1"
        self._planning_group = "manipulator"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description=self._robot_ns + "/robot_description",
                                                      ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description=self._robot_ns + "/robot_description",
                                                          ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher(self._robot_ns + '/move_group/display_planned_path',
                                                             moveit_msgs.msg.DisplayTrajectory, queue_size=2)
        self._exectute_trajectory_client = actionlib.SimpleActionClient(self._robot_ns + '/execute_trajectory',
                                                                        moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        rospy.set_param('/ur5_1_vacuum_gripper_service', False)

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._computed_plan = ''
        self._curr_state = self._robot.get_current_state()
        self._group.set_planning_time(99)
        rp = rospkg.RosPack()
        self._pkg_path = rp.get_path('pkg_task5')
        self._file_path = self._pkg_path + '/config/saved_trajectories/'

        rospy.loginfo("Package Path: {}".format(self._file_path))
        rospy.loginfo('\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo('\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo('\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')
        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

        ## MQTT Client

        # Initialize Action Client
        self._ac = actionlib.ActionClient('/action_ros_iot',
                                          msgRosIotAction)

        param_config_iot = rospy.get_param('config_pyiot')

        # Store the ROS Topic to get the start message from bridge action server
        self._param_order_topic = param_config_iot['mqtt']['sub_cb_ros_topic']
        self._config_mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']

        # Subscribe to the desired topic and attach a Callback Funtion to it.
        rospy.Subscriber(self._param_order_topic, msgMqttSub, self.func_callback_orders)

        # Dictionary to Store all the goal handels
        self._goal_handles = {}
        self._orders = []
        self._package_colours = None
        
        # Wait for Action Server that will use the action - '/action_iot_ros' to start
        self._ac.wait_for_server()
        rospy.loginfo("Action server up, we can send goals.")

    ''' Get all data from incoming orders. '''
    def func_callback_orders(self, msg):

        rospy.loginfo('***Order received:'+ msg.message)
        order_msg = eval(msg.message)

        self._order_type = {'Medicine':['HP', 'Red', '450'],
                            'Clothes':['LP', 'Green', '150'],
                            'Food':['MP', 'Yellow', '250']}
        order_id = order_msg['order_id']
        order_time = order_msg['order_time']
        order_item = order_msg['item']
        order_priority = self._order_type[order_item][0]
        order_city = order_msg['city']
        order_lon = order_msg['lon']
        order_lat = order_msg['lat']
        order_cost = self._order_type[order_item][2]

        info = {'id':'IncomingOrders', 'Team Id':'VB#693', 'Unique Id':'RRCneYRC',
                'Order ID':order_id, 'Order Date and Time': order_time,
                'Item':order_item, 'Priority':order_priority,
                'Order Quantity':'1', 'City':order_city, 'Longitude':order_lon,
                "Latitude":order_lat, 'Cost':order_cost}
        message = str(info)
        goal_handle = self.send_goal_to_mqtt_client("spreadsheet", "pub",
                                                    self._config_mqtt_pub_topic, message)
        self._goal_handles['Order'] = goal_handle

        if info["Priority"] == 'HP':
            info["color"] = 'red'
            info["package_name"] = self.assignName('red')
            info["location_on_shelf"] = self.assignLoc(info["package_name"])
        elif info["Priority"] == 'MP':
            info["color"] = 'yellow'
            info["package_name"] = self.assignName('yellow')
            info["location_on_shelf"] = self.assignLoc(info["package_name"])
        else:
            info["color"] = 'green'
            info["package_name"] = self.assignName('green')
            info["location_on_shelf"] = self.assignLoc(info["package_name"])

        self._orders.append(info)
        rospy.loginfo('******Orders Received******* :')
        self._original_orders.append(info)

    ''' Assigns package names to the prioritized order. '''
    def assignName(self, curr_color):
        for k in sorted(self._package_colours):
            if self._package_colours[k] == curr_color:
                val = k
                self._package_colours.pop(k)
                return val

    ''' Assigns package names to the prioritized order. '''
    def assignLoc(self, pkgName):
        val = self.packages_name_position[pkgName]
        self.packages_name_position.pop(pkgName)
        return val

    ''' Function to prioritize incoming order using insertion sort algorithm. '''
    def func_prioritize_orders(self):
        orders = self._orders
        l = len(orders)
        priority_to_value = {"LP":1, "MP":2, "HP":3}
        for i in range(l):
            pos = i
            while pos > 0 and priority_to_value[orders[pos]['Priority']] > priority_to_value[orders[pos-1]['Priority']]:
                orders[pos-1], orders[pos] = orders[pos], orders[pos-1]
                pos -= 1

        return orders

    ''' Function to get detected packages. '''     
    def camera1_callback(self):
        get_packages_type = rospy.ServiceProxy('/2Dcamera_packages_type', camera_packages)
        try:
            self.get_packages = get_packages_type(True)
            self._package_colours = eval(self.get_packages.pack_type)

        except rospy.ServiceException as exc:
            print "Service did not process request: " + str(exc)

    '''This function will be called when there is a change of state
       in the Action Client State Machine. '''
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

    ''' This function is used to send Goals to MQtt client. '''
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

    ''' Function to attach box to UR5_1 vacuum gripper. '''
    def attach_box(self, current_package, timeout=4):

        touch_links = self._robot.get_link_names(self._planning_group)
        self._scene.attach_box(self._eef_link, current_package, touch_links=touch_links)

        if rospy.get_param('/ur5_2_vacuum_gripper_service') == True or rospy.get_param('/conveyor_belt_service') == True:
            rospy.loginfo_once("Waiting for Service")
            rospy.sleep(1)

        rospy.set_param('/ur5_1_vacuum_gripper_service', True)
        try:
            rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
            self.attach = rospy.ServiceProxy('eyrc/vb/ur5/activate_vacuum_gripper/ur5_1',
                                             vacuumGripper)
            self.attach(True)
            rospy.set_param('/ur5_1_vacuum_gripper_service', False)

        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            print "Trying to reconnect service"
            rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
            rospy.set_param('/ur5_1_vacuum_gripper_service', True)
            self.attach = rospy.ServiceProxy('eyrc/vb/ur5/activate_vacuum_gripper/ur5_1',
                                             vacuumGripper)
            self.attach(True)
            rospy.set_param('/ur5_1_vacuum_gripper_service', False)

    ''' Function to detach box from UR5_1 vacuum gripper. '''
    def detach_remove_box(self, current_package, timeout=4):
        if rospy.get_param('/ur5_2_vacuum_gripper_service') == True or rospy.get_param('/conveyor_belt_service') == True:
            rospy.loginfo_once("Waiting for Service")
            rospy.sleep(1)
        rospy.set_param('/ur5_1_vacuum_gripper_service', True)
        self._scene.remove_attached_object(self._eef_link, name=current_package)
        try:
            rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
            self.attach = rospy.ServiceProxy('eyrc/vb/ur5/activate_vacuum_gripper/ur5_1',
                                             vacuumGripper)
            self.attach(False)
            rospy.set_param('/ur5_1_vacuum_gripper_service', False)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            print "Trying again to connect service"
            rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
            rospy.set_param('/ur5_1_vacuum_gripper_service', True)
            self.attach = rospy.ServiceProxy('eyrc/vb/ur5/activate_vacuum_gripper/ur5_1',
                                             vacuumGripper)
            self.attach(False)
            rospy.set_param('/ur5_1_vacuum_gripper_service', False)
        self._scene.remove_world_object(current_package)

    ''' Set UR5_1 joint angles. '''
    def set_joint_angles(self, arg_list_joint_angles):

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._computed_plan = self._group.plan()
        self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        flag_plan = self._group.go(wait=True)
        self._group.stop()

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

    ''' Function to play saved trajectories. ''' 
    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
        file_path = arg_file_path + arg_file_name
        list_joint_values = self._group.get_current_joint_values()
        with open(file_path, 'r') as file_open:
            loaded_plan = yaml.load(file_open)

        ret = self._group.execute(loaded_plan)
        return ret

    ''' Function to play saved trajectories until it reaches max attempt. '''
    def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):
        number_attempts = 0
        flag_success = False

        while ((number_attempts <= arg_max_attempts) and (flag_success is False)):
            number_attempts += 1
            flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
            rospy.logwarn("attempts: {}".format(number_attempts))

        return True

    ''' UR5_1 robot arm pick and place routine. '''
    def robot_pick_place(self, _current_package, package_location, conveyor_location, ordersheet):
        self.hard_set_joint_angles(package_location, 7)
        self.attach_box(_current_package)
        rospy.sleep(0.5)
        self.moveit_hard_play_planned_path_from_file(self._file_path,
                                                     'conveyor_'+_current_package+'.yaml', 5)
        self.detach_remove_box(_current_package)
        rospy.sleep(0.5)
        dt_string = datetime.now().strftime("%d/%m/%Y %H:%M:%S")  
        info = {'id':'OrdersDispatched', 'Team Id':'VB#693', 'Unique Id':'RRCneYRC',
                'Order ID':ordersheet['Order ID'],
                'City':ordersheet['City'], 'Item':ordersheet['Item'],
                'Priority':ordersheet['Priority'],
                'Dispatch Quantity':'1', 'Cost':ordersheet['Cost'],
                'Dispatch Status':'YES', 'Dispatch Date and Time': dt_string}
        message = str(info)
        goal_handle = self.send_goal_to_mqtt_client("spreadsheet", "pub",
                                                    self._config_mqtt_pub_topic, message)
        self._goal_handles['Order Dispatched'] = goal_handle
        self.publish_orders.publish(message)

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

''' Main Function. '''
def main():

    # Wait for Initializing Gazebo and Rviz
    # Create UR5 object
    ur5_1 = Ur5PickPlace()
   
    pkg_names=ur5_1.camera1_callback()
    
    # Initially move the robot to home position
    ur5_1.set_joint_angles(ur5_1.ur5_1_conveyor_pose)
    while not rospy.is_shutdown():

        if len(ur5_1._orders) != 0:
            ur5_1.func_prioritize_orders()
            curr_order = ur5_1._orders.pop(0)
            ur5_1.robot_pick_place(curr_order['package_name'], curr_order['location_on_shelf'],
                                   ur5_1.ur5_1_conveyor_pose, curr_order)
        else:
            pass

if __name__ == '__main__':
    main()

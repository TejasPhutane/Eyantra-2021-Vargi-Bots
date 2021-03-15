#!/usr/bin/env python

'''ROS Node - Action Server - ROS IOT Bridge'''

import threading
import rospy
import actionlib

from pkg_ros_iot_bridge.msg import msgRosIotAction
# Message Class that is used by ROS Actions internally

from pkg_ros_iot_bridge.msg import msgRosIotResult
# Message Class that is used for Action Result Messages

from pkg_ros_iot_bridge.msg import msgMqttSub
# Message Class for MQTT Subscription Messages

from pyiot import iot
#Custom Python Module to perfrom MQTT Tasks

'''This class is used for initializing the action server for Ros-IoT Bridge.'''
class IotRosBridgeActionServer:
    
    '''it is used for initializing the Action Server.'''
    # Constructor
    def __init__(self):
        # Initialize the Action Server
        self._as = actionlib.ActionServer('/action_ros_iot',
                                          msgRosIotAction,
                                          self.on_goal,
                                          self.on_cancel,
                                          auto_start=False)

        '''
        * self.on_goal - It is the fuction pointer which points to a function which will be called
                         when the Action Server receives a Goal.
        * self.on_cancel - It is the fuction pointer which points to a function which will be called
                           when the Action Server receives a Cancel Request.
        '''

        # Read and Store IoT Configuration data from Parameter Server
        param_config_iot = rospy.get_param('config_pyiot')
        self._config_mqtt_server_url = param_config_iot['mqtt']['server_url']
        self._config_mqtt_server_port = param_config_iot['mqtt']['server_port']
        self._config_mqtt_sub_topic = param_config_iot['mqtt']['topic_sub']
        self._online_orders_sub_topic = param_config_iot['mqtt']['topic_order']
        self._config_mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']
        self._config_mqtt_qos = param_config_iot['mqtt']['qos']
        self._config_mqtt_sub_cb_ros_topic = param_config_iot['mqtt']['sub_cb_ros_topic']
        self._config_sheet_url = param_config_iot['google_apps']['spread_sheet_id']
        self._config_submission_sheet_url = param_config_iot['google_apps']['submission_spread_sheet_id']
        print param_config_iot


        # Initialize ROS Topic Publication
        # Incoming message from MQTT Subscription will be published on a ROS Topic
        #(/ros_iot_bridge/mqtt/sub).
        # ROS Nodes can subscribe to this ROS Topic (/ros_iot_bridge/mqtt/sub) to get messages
        #from MQTT Subscription.
        self._handle_ros_pub = rospy.Publisher(self._config_mqtt_sub_cb_ros_topic,
                                               msgMqttSub, queue_size=10)


        # Subscribe to MQTT Topic (eyrc/RRCneYRC/iot_to_ros),
        #which is defined in 'config_iot_ros.yaml'.
        # self.mqtt_sub_callback() function will be called when,
        #there is a message from MQTT Subscription.
        ret = iot.mqtt_subscribe_thread_start(self.mqtt_sub_callback,
                                              self._config_mqtt_server_url,
                                              self._config_mqtt_server_port,
                                              self._online_orders_sub_topic,
                                              self._config_mqtt_qos)
        if ret == 0:
            rospy.loginfo("MQTT Subscribe Thread Started")
        else:
            rospy.logerr("Failed to start MQTT Subscribe Thread")

        # Start the Action Server
        self._as.start()
        rospy.loginfo("Started ROS-IoT Bridge Action Server.")


    ''' This is a callback function for MQTT Subscriptions. '''
    def mqtt_sub_callback(self, client, userdata, message):

        payload = str(message.payload.decode("utf-8"))

        print("[MQTT SUB CB] Message: ", payload)
        print("[MQTT SUB CB] Topic: ", message.topic)

        msg_mqtt_sub = msgMqttSub()
        msg_mqtt_sub.timestamp = rospy.Time.now()
        msg_mqtt_sub.topic = message.topic
        msg_mqtt_sub.message = payload

        self._handle_ros_pub.publish(msg_mqtt_sub)

    ''' This function will be called when Action Server receives a Goal. '''
    def on_goal(self, goal_handle):

        goal = goal_handle.get_goal()

        rospy.loginfo("Received new goal from Client")
        rospy.loginfo(goal)

        # Validate incoming goal parameters

        if goal.protocol == "spreadsheet":
            goal_handle.set_accepted()
            # Start a new thread to push messages to Google Sheets
            thread_sheet = threading.Thread(name="worker_sheet",
                                            target=self.publish_to_spreadsheet,
                                            args=(goal.message,))
            thread_sheet.start()

        elif goal.protocol == "mqtt":
            if((goal.mode == "pub") or (goal.mode == "sub")):
                goal_handle.set_accepted()
                # Start a new thread to process new goal from the client
                #(For Asynchronous Processing of Goals)
                # 'self.process_goal' - is the function pointer
                #which points to a function that will process incoming Goals
                thread = threading.Thread(name="worker",
                                          target=self.process_goal,
                                          args=(goal_handle,))
                thread.start()

            else:
                goal_handle.set_rejected()
                return
        else:
            goal_handle.set_rejected()
            return

    '''Function for publishing goal to google spreadsheets'''
    def publish_to_spreadsheet(self, goal):
        rospy.loginfo(" Sending result to spreadsheet!")
        values = eval(goal)
        iot.push_to_google_sheet(self._config_sheet_url, values)
        iot.push_to_google_sheet(self._config_submission_sheet_url, values)


    '''This function is called is a separate thread to process Goal.'''
    def process_goal(self, goal_handle):

        flag_success = False
        result = msgRosIotResult()

        goal_id = goal_handle.get_goal_id()
        rospy.loginfo("Processing goal : " + str(goal_id.id))

        goal = goal_handle.get_goal()

        # Goal Processing
        if goal.protocol == "mqtt":
            rospy.logwarn("MQTT")

            if goal.mode == "pub":
                rospy.logwarn("MQTT PUB Goal ID: " + str(goal_id.id))

                rospy.logwarn(goal.topic + " > " + goal.message)

                ret = iot.mqtt_publish(self._config_mqtt_server_url,
                                       self._config_mqtt_server_port,
                                       goal.topic,
                                       goal.message,
                                       self._config_mqtt_qos)

                if ret == 0:
                    rospy.loginfo("MQTT Publish Successful.")
                    result.flag_success = True
                else:
                    rospy.logerr("MQTT Failed to Publish")
                    result.flag_success = False

            elif goal.mode == "sub":
                rospy.logwarn("MQTT SUB Goal ID: " + str(goal_id.id))
                rospy.logwarn(goal.topic)

                ret = iot.mqtt_subscribe_thread_start(self.mqtt_sub_callback,
                                                      self._config_mqtt_server_url,
                                                      self._config_mqtt_server_port,
                                                      goal.topic,
                                                      self._config_mqtt_qos)
                if ret == 0:
                    rospy.loginfo("MQTT Subscribe Thread Started")
                    result.flag_success = True
                else:
                    rospy.logerr("Failed to start MQTT Subscribe Thread")
                    result.flag_success = False

        rospy.loginfo("Send goal result to client")
        if result.flag_success == True:
            rospy.loginfo("Succeeded")
            goal_handle.set_succeeded(result)
        else:
            rospy.loginfo("Goal Failed. Aborting.")
            goal_handle.set_aborted(result)

        rospy.loginfo("Goal ID: " + str(goal_id.id) + " Goal Processing Done.")


    def on_cancel(self, goal_handle):
        '''This function will be called when Goal Cancel request is send to the Action Server'''
        rospy.loginfo("Received cancel request.")
        #goal_id = goal_handle.get_goal_id()


'''Initialize '''
def main():
    rospy.init_node('node_action_server_ros_iot_bridge')

    # Create Iot Ros Bridge Action Server object
    action_server = IotRosBridgeActionServer()

    rospy.spin()



if __name__ == '__main__':
    main()

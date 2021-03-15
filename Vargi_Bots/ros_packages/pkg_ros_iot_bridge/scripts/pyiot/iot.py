from multiprocessing.dummy import Pool
import time
import sys
import requests
import paho.mqtt.client as mqtt #import the client1

class print_colour:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


# -----------------  MQTT SUB -------------------
def iot_func_callback_sub(client, userdata, message):  #Printing all the messages

    print("message received ", str(message.payload.decode("utf-8")))
    print("message topic=", message.topic)
    print("message qos=", message.qos)
    print("message retain flag=", message.retain)
    
    print(iot_func_callback_sub.__doc__)

def mqtt_subscribe_thread_start(arg_callback_func, arg_broker_url, arg_broker_port,   #Subscribing to MQTT
	                               arg_mqtt_topic, arg_mqtt_qos):
    try:
        mqtt_client = mqtt.Client()
        mqtt_client.on_message = arg_callback_func
        mqtt_client.connect(arg_broker_url, arg_broker_port)
        mqtt_client.subscribe(arg_mqtt_topic, arg_mqtt_qos)
        time.sleep(1) # wait
        #mqtt_client.loop_forever()
        #starts a blocking infinite loop
        mqtt_client.loop_start()    
        #starts a new thread
        return 0
    except:
        return -1
    #print("mqtt subscribe thread start")

# -----------------  MQTT PUB -------------------
def mqtt_publish(arg_broker_url, arg_broker_port, arg_mqtt_topic, arg_mqtt_message, arg_mqtt_qos):
   #Publishing to MQTT Client
    try:
        mqtt_client = mqtt.Client("mqtt_pub")
        mqtt_client.connect(arg_broker_url, arg_broker_port)
        mqtt_client.loop_start()

        print("Publishing message to topic", arg_mqtt_topic)
        mqtt_client.publish(arg_mqtt_topic, arg_mqtt_message, arg_mqtt_qos)
        time.sleep(0.1) # wait

        mqtt_client.loop_stop() #stop the loop
        return 0
    except:
        return -1
    print(mqtt_publish.__doc__)
#-----------------  Google Spreadsheet -------------------

def push_to_google_sheet(spreadsheet_url, parameters):
    URL = "https://script.google.com/macros/s/" + spreadsheet_url + "/exec"
    response = requests.get(URL, params=parameters)
    print response.content

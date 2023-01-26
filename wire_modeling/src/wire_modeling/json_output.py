#!/usr/bin/env python3

import rospy
# from sensor_msgs.msg import Image, CameraInfo
# from cv_bridge import CvBridge,CvBridgeError
# import cv2
# import numpy as np 
# import matplotlib.pyplot as plt
import json
from datetime import datetime
from control_msgs.msg import JointTrajectoryControllerState
from time import sleep
from threading import Thread

class JSONOutput:
    def __init__(self):
        self.json_dict = [{"_id":"/a_bot_/joint_states", "waypoints":[]},
                          {"_id":"/b_bot_/joint_states", "waypoints":[]}]

        # Create subscribers to gather data for export
        self.a_bot_status_sub = rospy.Subscriber("/a_bot_/arm_controller/state", JointTrajectoryControllerState, self.data_callback, callback_args={"name":"/a_bot_/joint_states", "dict_index":0})
        self.b_bot_status_sub = rospy.Subscriber("/b_bot_/arm_controller/state", JointTrajectoryControllerState, self.data_callback, callback_args={"name":"/b_bot_/joint_states", "dict_index":1})

    def data_callback(self, data, callback_args):
        # print(data.actual)
        # data = dict(data)
        new_waypoint = dict()
        new_waypoint["timestamp"] = {
            "nsec": data.actual.time_from_start.nsecs,
            "sec": data.actual.time_from_start.secs
        }
        new_waypoint["smoothness"] = 0 # 0 for now, no smoothness value
        new_waypoint["pose"] = {
            "position": {
                "x": data.actual.positions[0],
                "y": data.actual.positions[1],
                "z": data.actual.positions[2]
            },
            "orientation": {
                "x": data.actual.positions[3],
                "y": data.actual.positions[4],
                "z": data.actual.positions[5],
                "w": 0.0 # data["actual"]["positions"][6]
            }
        }

        self.json_dict[callback_args["dict_index"]]["waypoints"].append(new_waypoint)
        print(self.json_dict)

    def id_in_json(self, id) -> int:
        # Return index of matching id in list of dictionaries
        for d_i in range(len(self.json_dict)):
            if id in self.json_dict[d_i].keys():
                return d_i
        return -1

    def add_waypoint(self, id, wire_grasp_pose, time):
        # Perform obj conversion to JSON acceptable form here
        id_index = self.id_in_json(id)
        if id_index < 0: # id is not in obj, add a new id to going list of dict to export
            self.json_dict.append({
                "_id": str(id),
                "waypoints":[]
            })
            id_index = len(self.json_dict)-1
        
        new_waypoint = dict()
        split_time = str(time).split(".")
        new_waypoint["timestamp"] = {
            "nsec": int(split_time[1]) * 1e+9,
            "sec": int(split_time[0])
        }
        new_waypoint["smoothness"] = 0 # 0 for now, no smoothness value
        new_waypoint["pose"] = {
            "position": {
                "x": wire_grasp_pose.position.x,
                "y": wire_grasp_pose.position.y,
                "z": wire_grasp_pose.position.z
            },
            "orientation": {
                "x": wire_grasp_pose.orientation.x,
                "y": wire_grasp_pose.orientation.y,
                "z": wire_grasp_pose.orientation.z,
                "w":  wire_grasp_pose.orientation.w
            }
        }

        self.json_dict[id_index]["waypoints"].append(new_waypoint)
        print("JSON_DICT: ", self.json_dict)
        # return self.json_result

    def export_json(self):
        # print("hit export_json")
        datetime_str = (datetime.now()).strftime("%m-%d-%Y_%H-%M-%S")
        formatted_filename = "arm-trajectories_" + datetime_str + ".json"
        # print(datetime_str, formatted_filename)
        while True:
            with open(formatted_filename, "w") as outfile:
                json.dump(self.json_dict, outfile, indent=4)
                sleep(10)
            # print("dumped")
            # json.dump({"a":"test"}, outfile, indent=4)
        # print("finish export_json")

        # NEXT STEPS: file not created

# NODE FOR DOCUMENTING CURRENT POSITION OF ARMS
if __name__ == "__main__":
    print("Exporting JSON of planned trajectory")
    rospy.init_node('json_output')
    json_exporter = JSONOutput()

    try:
        rospy.spin()
        # Thread(target=json_exporter.export_json()).start()
         # This must happen at very end
    except KeyboardInterrupt:
        print("shut down")

    print("end of program")
    json_exporter.export_json()

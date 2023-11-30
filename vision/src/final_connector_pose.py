#!/usr/bin/env python3
from time import sleep
import rospy

# Transform publishing
from tf.transformations import quaternion_from_euler
import tf2_ros
from geometry_msgs.msg import TransformStamped
import math

def transform_connector_grasp(child_name: str, source: str, pos_adj, ori_adj) -> None:
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = source
        t.child_frame_id = "{}".format(child_name)

        t.transform.translation.x = pos_adj[0] # Offset arm to right by value meters
        t.transform.translation.y = pos_adj[1]
        t.transform.translation.z = pos_adj[2] # Too close to wall, move back .05m

        q = quaternion_from_euler(ori_adj[0], ori_adj[1], ori_adj[2]) # pos_adj

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        br.sendTransform(t)

def main():
    rospy.init_node("final_connector_pose",anonymous=True)
    rospy.sleep(3)
    rate = rospy.Rate(60)
    rear_cam_spec = "mounted_cam"

    while not rospy.is_shutdown():
        ### Rear mounted cam transforms
        transform_connector_grasp(f"final_pose_{rear_cam_spec}", f"connector_d415_color_frame", [0, 0, 0], [-math.pi/2, 0, 0, 1])
        # # create prepose here
        transform_connector_grasp(f"final_prepose_{rear_cam_spec}", f"connector_d415_color_frame", [-0.15, 0, 0], [-math.pi/2, 0, 0, 1])

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rospy

# tf2 and Transformations
import math
import tf2_ros
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler

# Camera capture
import cv2
from cv_bridge import CvBridge

def transform_arm_camera_frame(parent_arm: str, pos_adj, ori_adj) -> None:
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "{}_bot_ee_arm_link".format(parent_arm) # parent string
    t.child_frame_id = "arm_camera_link"

    t.transform.translation.x = ori_adj[0]
    t.transform.translation.y = ori_adj[1]
    t.transform.translation.z = ori_adj[2]

    q = quaternion_from_euler(pos_adj[0], pos_adj[1], pos_adj[2]) # pos_adj
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

def main():
    # This node represents the arm mounted camera - responsible for:
    # - Broadcasting frame, offset from arm it is mounted on
    # - Publishing live images for aruco detection in separate node
    rospy.init_node('arm_camera_link')
    mounted_arm = "b"
    camera_pub = rospy.Publisher('arm_camera_images', Image, queue_size=10)
    camera_src = cv2.VideoCapture(6)
    img_bridge = CvBridge()
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # OFFSETS: [+forward/-backward, +left/-right, +up/-down]
        transform_arm_camera_frame(mounted_arm, [math.pi/2, math.pi/2, 0], [0, (0.1778 if mounted_arm == "b" else -0.1778), 0.05])
        
        ret, frame = camera_src.read()
        if ret == True:
            camera_pub.publish(img_bridge.cv2_to_imgmsg(frame, encoding='rgb8'))
                        
        rate.sleep()


#*** Node Starts Here ***#
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

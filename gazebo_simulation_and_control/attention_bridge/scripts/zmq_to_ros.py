#!/usr/bin/env python3
import rospy, zmq, json
from std_msgs.msg import Float32

if __name__=='__main__':
    print("=== ZMQ→ROS bridge starting ===")
    rospy.init_node('zmq_attention_bridge')
    rospy.loginfo("ROS node initialized")
    pub = rospy.Publisher('/attention/global', Float32, queue_size=10)
    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.connect("tcp://host.docker.internal:5557")
    print("Connected to ZMQ at tcp://host.docker.internal:5557")
    sub.setsockopt_string(zmq.SUBSCRIBE, "")

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        try:
            msg = sub.recv_json(flags=zmq.NOBLOCK)
            print("Received ZMQ:", msg)
            val = float(msg['attention'].get('global', 0.0) or 0.0)
            pub.publish(val)
            rospy.loginfo(f"Published /attention/global = {val}")
        except zmq.Again:
            pass
        rate.sleep()

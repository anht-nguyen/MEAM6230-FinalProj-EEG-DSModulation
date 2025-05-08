#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
import math

def main():
    rospy.init_node('modulation_simulator')
    pub = rospy.Publisher('/modulation_input', Float32, queue_size=1)
    rate_hz = rospy.get_param('~rate', 20)          # how fast we publish
    freq = rospy.get_param('~sin_freq', 0.2)        # oscillation freq (Hz)
    amp = rospy.get_param('~amplitude', 0.5)        # amplitude around 0.5
    offset = rospy.get_param('~offset', 0.5)        # so signal ∈[0,1]
    rate = rospy.Rate(rate_hz)
    t0 = rospy.Time.now().to_sec()

    rospy.loginfo("Starting modulation simulator: sin(2π·{}·t)".format(freq))
    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec() - t0
        # sine wave between [offset-amp, offset+amp]
        u = offset + amp * math.sin(2.0 * math.pi * freq * t)
        # ensure within [0,1]
        u = max(0.0, min(1.0, u))
        pub.publish(Float32(u))
        rate.sleep()

if __name__ == '__main__':
    main()

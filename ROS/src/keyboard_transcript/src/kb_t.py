#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

avg, avd, arg, ard = 0, 0, 0, 0


def callback(data):

    lin = data.linear.x
    ang = data.angular.z

    avgf = Float32()
    avgf.data = 0.3*(2*lin - ang)

    avdf = Float32()
    avdf.data = 0.3*(2*lin + ang)

    argf = Float32()
    argf.data = 0.3*(2*lin - ang)

    ardf = Float32()
    ardf.data = 0.3*(2*lin + ang)

    avg.publish(avgf)
    avd.publish(avdf)
    arg.publish(argf)
    ard.publish(ardf)


def transcript():
    global avg, avd, arg, ard
    avg = rospy.Publisher('vrep_speed_motorAvG', Float32, queue_size=10)
    avd = rospy.Publisher('vrep_speed_motorAvD', Float32, queue_size=10)
    arg = rospy.Publisher('vrep_speed_motorArG', Float32, queue_size=10)
    ard = rospy.Publisher('vrep_speed_motorArD', Float32, queue_size=10)
    rospy.Subscriber("key_vel", Twist, callback)

    rospy.init_node('keyboard_transcript', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    try:
        transcript()
    except rospy.ROSInterruptException:
        pass

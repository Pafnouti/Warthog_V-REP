#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Float32
import numpy as np
from scipy.spatial.transform import Rotation as R

K = 10

def calcForce(data):
    pos = data.position.z
    r = R.from_quat([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
    force = Twist()

    f = np.array([0,0,0])

    if pos < 0.3:
        if pos > -0.3:
            norm = -pos/.3
            inte = (np.arcsin(norm) + norm*(np.sqrt(1-norm**2))) + np.pi/2
            f[2] = (.6**2) * inte * .24 * 1000
            f[1] = 0
            f[0] = 0
        else:
            f[2] = (.6**2) * np.pi * .24 * 1000
            f[1] = 0
            f[0] = 0

    f[2] *= K

    tr_f = r.inv().apply(f)

    force.linear.x = tr_f[0]
    force.linear.y = tr_f[1]
    force.linear.z = tr_f[2]

    return force

def callbackAvD(data):
    force = calcForce(data)
    forceAvG_pub.publish(force)

def callbackArD(data):
    force = calcForce(data)
    forceArD_pub.publish(force)

def callbackAvG(data):
    force = calcForce(data)
    forceAvG_pub.publish(force)

def callbackArG(data):
    force = calcForce(data)
    forceArG_pub.publish(force)

def transcript():
    global forceAvG_pub, forceAvD_pub, forceArG_pub, forceArD_pub
    forceAvG_pub = rospy.Publisher('buoyancyForceAvG', Twist, queue_size=10)
    forceAvD_pub = rospy.Publisher('buoyancyForceAvD', Twist, queue_size=10)
    forceArG_pub = rospy.Publisher('buoyancyForceArG', Twist, queue_size=10)
    forceArD_pub = rospy.Publisher('buoyancyForceArD', Twist, queue_size=10)
    rospy.Subscriber("vrep_wheel_AvD", Pose, callbackAvD)
    rospy.Subscriber("vrep_wheel_ArD", Pose, callbackArD)
    rospy.Subscriber("vrep_wheel_AvG", Pose, callbackAvG)
    rospy.Subscriber("vrep_wheel_ArG", Pose, callbackArG)

    rospy.init_node('buoyancy_node', anonymous=True)
    rate = rospy.Rate(100)  # 100hz

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    try:
        transcript()
    except rospy.ROSInterruptException:
        pass

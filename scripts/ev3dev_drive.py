import ev3dev.ev3 as ev3

import rospy

from std_msgs.msg import String
import geometry_msgs
import geometry_msgs.msg

left = ev3.LargeMotor("A")
right = ev3.LargeMotor("D")
b = 2.25

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "\n" + str(data))
    x_l = data.linear.x
    z_a = data.angular.z
    left_sp = (x_l - (1 - 2*(x_l<0))*z_a*b)
    right_sp = (x_l + (1 - 2*(x_l<0))*z_a*b)
    if x_l or z_a:
        rospy.loginfo("run: (%f,%f)" % (left_sp,right_sp))
        left.run_forever(duty_cycle_sp=left_sp)
        right.run_forever(duty_cycle_sp=right_sp)
    else:
        rospy.loginfo("stop: (0.0,0.0)")
        left.stop(stop_command="brake")
        right.stop(stop_command="brake")

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("cmd_vel", geometry_msgs.msg.Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

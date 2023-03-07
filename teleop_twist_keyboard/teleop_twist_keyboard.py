#!/usr/bin/env python

from __future__ import print_function
from math import fabs

import threading

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

import sys
from select import select

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


TwistMsg = Twist

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

anything else : stop

q/z : increase/decrease only linear speed +10/-10
w/x : increase/decrease only steer angle +5/-5

e/c : auto-drive mode / controller mode

CTRL-C to quit
"""

moveBindings = {
        'u':(1,0,0,1),
        'i':(1,0,0,0),
        'o':(1,0,0,-1),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'm':(-1,0,0,-1),
        ',':(-1,0,0,0),
        '.':(-1,0,0,1),
        'e':(0,0,1,0),
        'c':(0,1,0,0),
    }

speedBindings={
        'q':(10,0),
        'z':(-10,0),
        'w':(0,5),
        'x':(0,-5),
    }

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('teleop_cmd_vel', TwistMsg, queue_size = 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.steer_input = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()
	

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        # self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):

        if stamped:
            twist = twist_msg.twist
            twist_msg.header.stamp = rospy.Time.now()
            twist_msg.header.frame_id = twist_frame
        else:
            twist = twist_msg
        while not self.done:
            if stamped:
                twist_msg.header.stamp = rospy.Time.now()
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            steer_gap = self.th*self.turn - twist.angular.z
            if   (steer_gap > 0): self.steer_input += 1
            elif (steer_gap < 0): self.steer_input -= 1

            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            if   twist.linear.x >=  255: twist.linear.x =  255
            elif twist.linear.x <= -255: twist.linear.x = -255
            twist.linear.y = self.y
            twist.linear.z = self.z
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th*self.turn
            #if self.th != 0: twist.angular.z = self.steer_input
            #else:            twist.angular.z = 0; self.turn =0
            if   twist.angular.z >=  25: twist.angular.z =  25
            elif twist.angular.z <= -25: twist.angular.z = -25

            self.condition.release()

            # Publish.
            self.publisher.publish(twist_msg)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist_msg)


def getKey(settings, timeout):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    global twist_msg
    twist_msg = TwistMsg()

    settings = saveTerminalSettings()

    rospy.init_node('teleop_twist_keyboard')

    speed = rospy.get_param("~speed", 30)
    turn = rospy.get_param("~turn", 20)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.5)
    stamped = rospy.get_param("~stamped", False)
    twist_frame = rospy.get_param("~frame_id", '')
    if stamped:
        TwistMsg = TwistStamped

    pub_thread = PublishThread(repeat)

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, th, speed, turn)

        print(msg)
        print(vels(speed,turn))
        while(1):
            key = getKey(settings, key_timeout)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed + speedBindings[key][0]
                if speed <= 0: speed = 0
                elif speed >= 255: speed = 255
                turn = turn + speedBindings[key][1]
                if   turn >=  25: turn =  25
                elif turn <= 5: turn = 5

                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
                    continue
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

            pub_thread.update(x, y, z, th, speed, turn)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        restoreTerminalSettings(settings)
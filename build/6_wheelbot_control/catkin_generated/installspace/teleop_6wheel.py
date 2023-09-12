#!/usr/bin/env python3

from __future__ import print_function

import threading

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from std_msgs.msg import Int16

import sys
from select import select

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

msg = """
Reading from the keyboard and Publishing to RPM Listener Topic!
---------------------------
Moving around:
   w: Move Forward
   x: Move Backward
   a: Turn Left
   d: Turn Right
   s: Stop

q/z: Increase/Decrease RPM by 10%

CTRL-C to quit
"""

moveBindings = {
    's': 0,  # Stop the robot with 's' key
    'w': 1,  # Map 'w' key to move forward
    'x': 2,  # Map 'x' key to move backward
    'd': 3,  # Map 'd' key to turn right
    'a': 4,  # Map 'a' key to turn left
    
    
}

class PublishThread(threading.Thread):
    def __init__(self):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('rpm_listener_topic', Int16, queue_size=1)
        self.data = 0
        self.condition = threading.Condition()
        self.done = False
        self.current_key = None

        self.start()

    def update(self, data):
        self.condition.acquire()
        self.data = data
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0)
        self.join()

    def run(self):
        while not self.done:
            self.condition.acquire()
            # Wait for a new message.
            self.condition.wait()

            # Publish RPM data.
            self.publisher.publish(self.data)

            self.condition.release()

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

if __name__=="__main__":
    settings = saveTerminalSettings()

    rospy.init_node('teleop_twist_keyboard')

    pub_thread = PublishThread()

    try:
        pub_thread.update(0)  # Initialize RPM to 0
        print(msg)
        while not rospy.is_shutdown():
            key = getKey(settings, 0.5)  # Timeout set to 0.1 seconds

            if key in moveBindings.keys():
                if pub_thread.current_key != key:
                    # Key was just pressed, update RPM
                    rpm_value = moveBindings[key]
                    rospy.loginfo("Pressed key: %s, RPM value: %d", key, rpm_value)
                    pub_thread.update(int(rpm_value))  # Convert RPM value to integer
                    pub_thread.current_key = key
            elif key == 'q':
                # Increase RPM by 10%
                rpm_value = max(0, pub_thread.data * 1.1)
                rospy.loginfo("Increased RPM to: %d", rpm_value)
                pub_thread.update(int(rpm_value))  # Convert RPM value to integer
            elif key == 'z':
                # Decrease RPM by 10%
                rpm_value = max(0, pub_thread.data * 0.9)
                rospy.loginfo("Decreased RPM to: %d", rpm_value)
                pub_thread.update(int(rpm_value))  # Convert RPM value to integer
            elif key == '\x03':
                # CTRL-C to quit
                break
            elif pub_thread.current_key:
                # Key was released, stop the robot
                rospy.loginfo("Released key: %s, stopping the robot", pub_thread.current_key)
                pub_thread.update(0)  # Stop the robot
                pub_thread.current_key = None

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        restoreTerminalSettings(settings)


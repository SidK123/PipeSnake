import rospy

class PipeSnake_Joystick_Interface():
    def init(self):
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
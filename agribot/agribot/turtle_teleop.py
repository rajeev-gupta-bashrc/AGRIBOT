import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import sys, select, os
if os.name == 'nt':
    import msvcrt
else:
    import tty, termios
if os.name != 'nt':
    settings = termios.tcgetattr(sys.stdin)


class Sender(Node):
    def __init__(self):
        super().__init__('command_sender')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'cmd', 10)
        timer_period = 0.01  # seconds
        self.cmd_data = Float32MultiArray()
        self.cmd_data.data = [0.0, 0.0]
        self.status = 0
        self.left_vel = 0.0
        self.right_vel = 0.0
        self.target_linear_vel   = 0.0
        self.target_angular_vel  = 0.0
        self.control_linear_vel  = 0.0
        self.control_angular_vel = 0.0
        self.MAX_LIN_VEL = 255
        self.MAX_ANG_VEL = 200
        self.LIN_VEL_STEP_SIZE = 5
        self.ANG_VEL_STEP_SIZE = 3
        self.msg = """
            Control Your Robot!
            ---------------------------
            Moving around:
                    w
               a    s    d
                    x
            
            w/x : increase/decrease linear velocity 
            a/d : increase/decrease angular velocity
            
            space key, s : force stop
            
            CTRL-C to quit
            """
        print(self.msg)
        self.timer = self.create_timer(timer_period, self.loop)
    
    def vels(self, target_linear_vel, target_angular_vel):
        return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

    def makeSimpleProfile(self, output, input, slop):
        if input > output:
            output = min( input, output + slop )
        elif input < output:
            output = max( input, output - slop )
        else:
            output = input

        return output

    def constrain(self, input, low, high):
        if input < low:
          input = low
        elif input > high:
          input = high
        else:
          input = input

        return input

    def checkLinearLimitVelocity(self, vel):
        vel = self.constrain(vel, -self.MAX_LIN_VEL, self.MAX_LIN_VEL)
        return vel

    def checkAngularLimitVelocity(self, vel):
        vel = self.constrain(vel, -self.MAX_ANG_VEL, self.MAX_ANG_VEL)
        return vel

    def getKey(self):
        if os.name == 'nt':
          if sys.version_info[0] >= 3:
            return msvcrt.getch().decode()
          else:
            return msvcrt.getch()
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def loop(self):
        try:
            key = self.getKey()
            if key == 'w' :
                self.target_linear_vel = self.checkLinearLimitVelocity(self.target_linear_vel + self.LIN_VEL_STEP_SIZE)
                self.status += 1
                print(self.vels(self.target_linear_vel, self.target_angular_vel))
            elif key == 'x' :
                self.target_linear_vel = self.checkLinearLimitVelocity(self.target_linear_vel - self.LIN_VEL_STEP_SIZE)
                self.status +=1
                print(self.vels(self.target_linear_vel, self.target_angular_vel))
            elif key == 'a' :
                self.target_angular_vel = self.checkAngularLimitVelocity(self.target_angular_vel + self.ANG_VEL_STEP_SIZE)
                self.status +=1
                print(self.vels(self.target_linear_vel, self.target_angular_vel))
            elif key == 'd' :
                self.target_angular_vel = self.checkAngularLimitVelocity(self.target_angular_vel - self.ANG_VEL_STEP_SIZE)
                self.status +=1
                print(self.vels(self.target_linear_vel, self.target_angular_vel))
            elif key == ' ' or key == 's' :
                self.target_linear_vel   = 0.0
                self.control_linear_vel  = 0.0
                self.target_angular_vel  = 0.0
                self.control_angular_vel = 0.0
                print(self.vels(self.target_linear_vel, self.target_angular_vel))
            elif key == '\x03':
                print("Stopping the bot")
                raise KeyboardInterrupt("KeyBoard interrupted")
            else:
                pass
            if self.status >= 20 :
                print(self.msg)
                self.status = 0
            self.control_linear_vel = self.makeSimpleProfile(self.control_linear_vel, self.target_linear_vel, (self.LIN_VEL_STEP_SIZE/2.0))
            self.control_angular_vel = self.makeSimpleProfile(self.control_angular_vel, self.target_angular_vel, (self.ANG_VEL_STEP_SIZE/2.0))
            self.left_vel = self.checkLinearLimitVelocity(self.control_linear_vel - self.control_angular_vel)
            self.right_vel = self.checkLinearLimitVelocity(self.control_linear_vel + self.control_angular_vel)
        except KeyboardInterrupt as e:
            print(e)
            self.timer.cancel()
            print("stopped timer")
            self.left_vel = 0.0
            self.right_vel = 0.0
            self.cmd_data.data[0] = 0.0
            self.cmd_data.data[1] = 0.0
        finally:
            self.cmd_data.data[0] = self.left_vel
            self.cmd_data.data[1] = self.right_vel
            self.publisher_.publish(self.cmd_data)



def main():
    rclpy.init(args=None)
    sender_node = Sender()
    rclpy.spin(sender_node)
    sender_node.destroy_node()
    rclpy.shutdown()
    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__=='__main__':
    main()


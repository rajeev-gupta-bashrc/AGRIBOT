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
        self.abs_vel = 0.0
        self.left_vel = 0.0
        self.right_vel = 0.0
        self.prev_key = ''
        self.P = 0.1
        self.msg = """
                    Control Your TurtleBot3!
                    ---------------------------
                    Moving around:
                            w
                       a    s    d
                            x
                    
                    w/x : Forward/Backward 
                    a/d : Left/Right inplace rotation
                    
                    space key, s : force stop
                    
                    CTRL-C to quit
                    """
        print(self.msg)
        self.timer = self.create_timer(timer_period, self.loop)
    
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
    
    def normalize(self, m):
        abm = abs(m)
        if abm <1.0:
            m = 1.0
        elif abm>255:
            m = 255
        return m
    
    def accel(self, l, r):
        l_error = l - self.left_vel
        r_error = r - self.right_vel
        self.left_vel += l_error*self.P
        self.right_vel += r_error*self.P
        self.left_vel = self.normalize(self.left_vel)
        self.right_vel = self.normalize(self.right_vel)

    def loop(self):
        try:
            key = self.getKey()
            if key == 'w' :
                self.accel(self.abs_vel, self.abs_vel)
            elif key == 'x' :
                self.accel(-self.abs_vel, -self.abs_vel)
            elif key == 'a' :
                self.accel(-self.abs_vel, self.abs_vel)
            elif key == 'd' :
                self.accel(self.abs_vel, -self.abs_vel)
            elif key == 's' :
                self.abs_vel = 0.0
                self.left_vel = 0.0
                self.right_vel = 0.0
                self.cmd_data.data[0] = 0.0
                self.cmd_data.data[1] = 0.0
            elif key == ',':
                if self.abs_vel > 1.0:
                    self.abs_vel*=0.9
                else:
                    self.abs_vel = 1.0
                self.status+=1
                print('Current speed: ', self.abs_vel)
            elif key == '.':
                if self.abs_vel < 1.0:
                    self.abs_vel=1.0
                elif self.abs_vel < 230.0:
                    self.abs_vel*=1.1
                else:
                    self.abs_vel = 255.0
                self.status+=1
                print('Current speed: ', self.abs_vel)
            elif key == '\x03':
                print("Stopping the bot")
                raise KeyboardInterrupt("KeyBoard interrupted")
            else:
                pass
            if self.status == 20 :
                print(self.msg)
                self.status = 0
        except Exception as e:
            print(e)
            self.timer.cancel()
            print("stopped timer")
            self.abs_vel = 0.0
            self.left_vel = 0.0
            self.right_vel = 0.0
            self.cmd_data.data[0] = 0.0
            self.cmd_data.data[1] = 0.0
        finally:
            if (key!='w' and key!='x' and key!='a' and key!='d' and key!='') or self.prev_key=='':
                self.cmd_data.data[0] = 0.0
                self.cmd_data.data[1] = 0.0
                self.left_vel = 0.0
                self.right_vel = 0.0
            else:
                self.cmd_data.data[0] = self.left_vel
                self.cmd_data.data[1] = self.right_vel
            self.publisher_.publish(self.cmd_data)
            self.prev_key = key



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


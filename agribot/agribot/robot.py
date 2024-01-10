import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM) 


class Motor():
    def __init__(self, dirpin, pwmpin):
        self.dirpin = dirpin
        self.pwmpin = pwmpin
        GPIO.setup(self.dirpin, GPIO.OUT)
        GPIO.setup(self.pwmpin, GPIO.OUT)
        self.signal = GPIO.PWM(self.pwmpin, 490)            #490 Hz frequency

    def run(self, m):
        abm = abs(m)
        if (m < 0):
            GPIO.output(self.dirpin, 1)
        if (m > 0):
            GPIO.output(self.dirpin, 0)
        self.signal.start(abm)

    def cleanup_(self):
        GPIO.output(self.dirpin, 0)  # Set direction pin to low
        GPIO.output(self.pwmpin, 0)  # Set PWM pin to low
        self.signal = GPIO.PWM(self.pwmpin, 0)
        self.signal.stop()  # Stop PWM signal
        GPIO.cleanup([self.dirpin, self.pwmpin])  # Clean up GPIO pins
        print("Destructor evoked")
        

class My_Robot():
    def __init__(self):
        self.motors = [Motor(2, 3), Motor(4, 14), Motor(15, 18), Motor(17, 27), Motor(22, 23)]
        self.left_wing_vel = 0.0
        self.right_wing_vel = 0.0

    def run_robot(self, l, r):
        self.left_wing_vel = l
        self.right_wing_vel = r
        self.motors[0].run(self.left_wing_vel)
        self.motors[2].run(self.left_wing_vel)
        self.motors[1].run(self.right_wing_vel)
        self.motors[3].run(self.right_wing_vel)
        self.motors[4].run(self.right_wing_vel)

    def cleanup_(self):
        for motor in self.motors:
            motor.cleanup_()

class Reciever(Node):

    def __init__(self):
        super().__init__('reciever')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'cmd',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.data = [0, 0]
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.loop)
        self.rover = My_Robot()

    def listener_callback(self, msg):
        self.data[0] = int(msg.data[0]*100/255)
        self.data[1] = int(msg.data[1]*100/255)
        self.get_logger().info('I heard: "%d, %d"' % (msg.data[0], msg.data[1]))

    def loop(self):
        l = self.data[0]
        r = self.data[1]
        print('I set (left_velocity, right_velocity): ', l, r)
        self.rover.run_robot(l, r)

    def cleanup_(self):
        self.rover.cleanup_()

def main(args=None):
    rclpy.init(args=args)

    reciever_node = Reciever()

    try:
        rclpy.spin(reciever_node)
    except KeyboardInterrupt:
        print("Bot interrupted")
        reciever_node.cleanup_()
    # reciever_node.cleanup_()
    reciever_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
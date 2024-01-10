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

    def __del__(self):
        GPIO.output(self.dirpin, 0)  # Set direction pin to low
        GPIO.output(self.pwmpin, 0)  # Set PWM pin to low
        self.signal.stop()  # Stop PWM signal
        GPIO.cleanup([self.dirpin, self.pwmpin])  # Clean up GPIO pins
        print("Destructor evoked")
        
motors = [Motor(2, 3), Motor(4, 14), Motor(15, 18), Motor(17, 27), Motor(22, 23)]

def run_left(m):
    motors[0].run(m)
    motors[2].run(m)

def run_right(m):
    motors[1].run(m)
    motors[3].run(m)
    motors[4].run(m)


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

    def listener_callback(self, msg):
        self.data[0] = int(msg.data[0]*100/255)
        self.data[1] = int(msg.data[1])
        self.get_logger().info('I heard: "%d, %d"' % (msg.data[0], msg.data[1]))

    def loop(self):
        v = self.data[0]
        d = self.data[1]
        print('I set (velocity, state): ', v, d)
        if d == 0:
            run_left(v)
            run_right(v)
        elif d == 1:
            run_left(-v)
            run_right(-v)
        elif d == 2:
            run_left(-v)
            run_right(v)
        elif d == 3:
            run_left(v)
            run_right(-v)
        else:
            run_left(0)
            run_right(0)


def main(args=None):
    rclpy.init(args=args)

    reciever_node = Reciever()

    try:
        rclpy.spin(reciever_node)
    except KeyboardInterrupt:
        print("Bot interrupted")        
        reciever_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
import os
import random
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from gtts import gTTS

class RatRoulette(Node):
    def __init__(self):
        super().__init__('rat_roulette')

        self.START = 0
        self.SPIN = 1
        self.ANNOUNCE = 2
        self.NAVIGATE = 3
        self.DETECT = 4
        self.REWARD = 5
        self.RESET = 6

        self.SPEED_LINEAR = 0.3
        self.SPEED_ANGULAR = 3.0

        self.image_width = 300
        self.last_detection = None

        self.state = self.START # Set initial state
        self.result = 0 #index of results

        self.r_time = 3.345
        self.s_time = self.r_time / 8
        self.offset = self.s_time / 2
        self.results = [[0, 1], [0, 2], [0, 3], [0, 4], [1, 1], [1, 2], [1, 3], [1, 4]]
        self.spin_times = [self.s_time * 31 + self.offset , self.s_time * 25 + self.offset, self.s_time * 27 + self.offset, self.s_time * 29 + self.offset, self.s_time * 26 + self.offset, self.s_time * 28 + self.offset, self.s_time * 30 + self.offset, self.s_time * 32 + self.offset] #TODO: Change when we have the times for each spin

        self.state_ts = self.get_clock().now()
        self.second = False

        # subscribe to YOLO pedestrian detections /color/mobilenet_detections
        self.person_detection_sub = self.create_subscription(
                Detection2DArray,
                '/color/mobilenet_detections',
                self.person_detection_callback,
                qos_profile_sensor_data)

        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.control_cycle)

    def person_detection_callback(self, msg):
        self.last_detection = msg
        #print(msg)

    def control_cycle(self):
        out_vel = Twist()
        if self.person_detected():
            print("yayyyyy, i see you!")
        print("curr state: ", self.state)

        if self.state == self.START:
            if self.last_detection is not None:
                print("camera active?")
                self.result = random.randint(0, 7)
                print("random result: ", self.result)
                self.go_state(self.SPIN)
            else:
                print("camera not working")
            #self.go_state(self.SPIN)
        elif self.state == self.SPIN:
            #os.system("mpg123 ~/rat-roulette/ros2_ws/src/rat_roulette_pkg/rat_roulette_pkg/win.mp3")
            if self.check_spin_time():
                out_vel.angular.z = 0.0
                self.go_state(self.ANNOUNCE)
            else:
                out_vel.angular.z = self.SPEED_ANGULAR
        elif self.state == self.ANNOUNCE:
            print("The result is... ", self.results[self.result])
            elapsed = self.get_clock().now() - self.state_ts
            if elapsed >= Duration(seconds=10):
                self.go_state(self.NAVIGATE)
        elif self.state == self.NAVIGATE:
            # navigate to answer
            if not self.second:
                pass
                # navigate to results[result[0]]
            else:
                #navigate to results[result[1]]
                pass
            self.go_state(self.DETECT)
        elif self.state == self.DETECT:
            if self.person_detected():
                print("Congrats!")
            else:
                print(":( No winnders")
            if not self.second:
                self.second = True
                self.go_state(self.NAVIGATE)
            else:
                self.second = False
                self.go_state(self.RESET)
        elif self.state == self.RESET:
            # go back to original position
            self.go_state(self.START)

        self.vel_pub.publish(out_vel)

    def go_state(self, new_state):
        self.state = new_state
        self.state_ts = self.get_clock().now()

    def person_detected(self):
        if self.last_detection is None:
            return False
        for detection in self.last_detection.detections:
            for result in detection.results:
                class_id = result.hypothesis.class_id
                if class_id == '15':
                    bbox = detection.bbox
                    bbox_center_x = bbox.center.position.x
                    image_center_x = self.image_width / 2

                    tolerance = self.image_width * 0.1

                    if abs(bbox_center_x - image_center_x) <= tolerance:
                        return True
        return False
    def check_spin_time(self):
        elapsed = self.get_clock().now() - self.state_ts
        print("elapsed time: ", elapsed)
        return elapsed >= Duration(seconds=self.spin_times[self.result]) #find out how much time for each answer
def main(args=None):
    print('Hi from rat_roulette_pkg.')
    rclpy.init(args=args)
    rat_roulette = RatRoulette()
    rclpy.spin(rat_roulette)

    rat_roulette.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


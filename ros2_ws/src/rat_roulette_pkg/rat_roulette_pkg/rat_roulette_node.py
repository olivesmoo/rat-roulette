import rclpy
import os
import random
import math
import threading

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
        self.MIDDLE = 7

        self.SPEED_LINEAR = 0.2
        self.SPEED_ANGULAR = 3.0

        self.image_width = 300
        self.last_detection = None

        self.state = self.START # Set initial state
        self.result = 0 #index of results

        self.r_time = 3.343 # time per rotation
        self.s_time = self.r_time / 8 # time per slice
        self.offset = self.s_time / 2
        self.results = [[0, 1], [0, 2], [0, 3], [0, 4], [1, 1], [1, 2], [1, 3], [1, 4]]
        self.spin_times = [self.s_time * 24 + self.offset , self.s_time * 26 + self.offset, self.s_time * 28 + self.offset, self.s_time * 30 + self.offset, self.s_time * 27 + self.offset - 0.1, self.s_time * 29 + self.offset, self.s_time * 31 + self.offset, self.s_time * 25 + self.offset - 0.1] #TODO: Change when we have the times for each spin
        self.origin_times = [self.offset, self.s_time * 2 + self.offset - 0.1, self.s_time * 3, self.s_time, self.offset, self.s_time + self.offset, self.s_time * 3 + self.offset, self.s_time*2 + 0.1]
        self.origin_directions = [-1, -1, 1, 1, 1, -1, -1, 1]

        self.number_times = [3, 1.5, 1.5, 3]
        self.color_times = [self.s_time * 7, self.s_time * 5, self.s_time * 3, self.s_time, self.s_time * 3 - self.offset, self.s_time - self.offset, self.s_time * 7 - self.offset, self.s_time * 5 - self.offset]
        self.state_ts = self.get_clock().now()
        self.second = False

        self.turn_90 = 0.8

        self.person = 0

        self.audio = False

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

        if self.state == self.START:
            if self.last_detection is not None:
                self.result = random.randint(0, 7)
                # self.result = 2
                print("random result: ", self.result, flush=True)
                self.play_audio("prediction.mp3")
                self.play_audio("waiting.mp3")
                self.go_state(self.SPIN)
            else:
                print("camera not working")
            #self.go_state(self.SPIN)
        elif self.state == self.SPIN:
            if not self.audio:
                if not hasattr(self, 'spin_audio_thread') or not self.spin_audio_thread.is_alive():
                    # Specify the audio file to play during the spin state
                    self.audio = True
                    spin_audio_file = "wheel3.mp3"
                    self.spin_audio_thread = threading.Thread(target=self.play_audio, args=(spin_audio_file,))
                    self.spin_audio_thread.start()

            if self.check_spin_time(self.spin_times[self.result]):
                self.audio = False
                out_vel.angular.z = 0.0
                self.go_state(self.ANNOUNCE)
            else:
                out_vel.angular.z = self.SPEED_ANGULAR
        elif self.state == self.ANNOUNCE:
            if  self.check_spin_time(1):
                if self.results[self.result][0] == 0:
                    audio_file = f"black{self.results[self.result][1]}.mp3"

                else:
                    audio_file = f"red{self.results[self.result][1]}.mp3"
                self.play_audio(audio_file)
                self.go_state(self.NAVIGATE)
        elif self.state == self.NAVIGATE:
            # navigate to answer
            if not self.second:
                if not self.check_spin_time(self.origin_times[self.result]):
                    out_vel.angular.z = self.SPEED_ANGULAR * (self.origin_directions[self.result])
                else:
                    out_vel.angular.z = 0.0
                    if not self.check_spin_time(self.origin_times[self.result] + 2.1):
                        out_vel.linear.x = self.SPEED_LINEAR
                    else:
                        out_vel.linear.x = 0.0
                        if not self.check_spin_time(self.origin_times[self.result] + 2.9):
                            if self.results[self.result][0] == 0:
                                out_vel.angular.z = self.SPEED_ANGULAR * -1
                            else:
                                out_vel.angular.z = self.SPEED_ANGULAR
                                
                        else:
                            if not self.check_spin_time(self.origin_times[self.result] + 5.2):
                                out_vel.linear.x = self.SPEED_LINEAR
                            else:
                                self.go_state(self.DETECT)
            else:
                # turn to left side
                number = self.results[self.result][1]
                multiplier = 1
                distance = 0
                if number == 1:
                    multiplier = -1
                    distance = 3.1
                elif number == 2:
                    multiplier = -1
                    distance = 1.2
                elif number == 3:
                    multiplier = 1
                    distance = 1.2
                elif number == 4:
                    distance = 3.1

                if not self.check_spin_time(0.8):
                    out_vel.angular.z = self.SPEED_ANGULAR * multiplier
                else:
                    if not self.check_spin_time(0.8 + distance):
                        out_vel.linear.x = self.SPEED_LINEAR
                        out_vel.angular.z = 0.0
                    else:
                        if not self.check_spin_time(1.6 + distance):
                            out_vel.angular.z = self.SPEED_ANGULAR * multiplier * -1
                            out_vel.linear.x = 0.0
                        else:
                            if not self.check_spin_time(3.9 + distance):
                                out_vel.linear.x = self.SPEED_LINEAR
                            else:
                                self.go_state(self.DETECT)
        elif self.state == self.DETECT:
            wait_time = 0
            if not self.person_detected() and self.check_spin_time(2.9):
                print(":( No winners", flush=True)
                wait_time = 2
            elif self.person_detected():
                print("Yes winners!!", flush=True)
                wait_time = 8
                self.person += 1
            if self.check_spin_time(2.0):
                if self.person >= 5:
                    if self.second:  
                        self.play_audio("twocheeses.mp3")
                    else:
                        self.play_audio("onecheese.mp3")
                    self.play_audio("clapping.mp3")
                else:
                    self.play_audio("gameover.mp3")
                self.person = 0
                if not self.second:
                    self.second = True
                    self.go_state(self.MIDDLE)
                else:
                    self.second = False
                    self.go_state(self.RESET)
        elif self.state == self.MIDDLE:
            multiplier = 1
            if self.results[self.result][0] == 0:
                multiplier = -1
            if not self.check_spin_time(0.8):
                out_vel.angular.z = self.SPEED_ANGULAR * multiplier
            else:
                if not self.check_spin_time(2.9):
                    out_vel.linear.x = self.SPEED_LINEAR
                    out_vel.angular.z = 0.0
                else:
                    if not self.check_spin_time(3.7):
                        out_vel.angular.z = self.SPEED_ANGULAR * multiplier
                        out_vel.linear.x = 0.0

                    else:
                        if not self.check_spin_time(6.0):
                            out_vel.linear.x = self.SPEED_LINEAR
                            out_vel.angular.z = 0.0
                        else:
                            self.go_state(self.NAVIGATE)

        elif self.state == self.RESET:
            number = self.results[self.result][1]
            multiplier = 1
            distance = 0
            if number == 1:
                multiplier = 1
                distance = 3.1
            elif number == 2:
                multiplier = 1
                distance = 1.2
            elif number == 3:
                multiplier = -1
                distance = 1.2
            elif number == 4:
                multiplier = -1
                distance = 3.1

            if not self.check_spin_time(0.8):
                out_vel.angular.z = self.SPEED_ANGULAR * multiplier
            else:
                if not self.check_spin_time(0.8 + distance):
                    out_vel.linear.x = self.SPEED_LINEAR
                    out_vel.angular.z = 0.0
                else:
                    if not self.check_spin_time(1.6 + distance):
                        out_vel.angular.z = self.SPEED_ANGULAR * multiplier
                        out_vel.linear.x = 0.0
                    else:
                        if not self.check_spin_time(3.9 + distance):
                            out_vel.linear.x = self.SPEED_LINEAR
                        else:
                            if not self.check_spin_time(4.7 + distance):
                                out_vel.angular.z = self.SPEED_ANGULAR
                            else:
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

                    bbox_width = bbox.size_x
                    bbox_height = bbox.size_y
                    min_bbox_size = 20


                    if abs(bbox_center_x - image_center_x) <= tolerance and \
                        bbox_width >= min_bbox_size and bbox_height >= min_bbox_size:
                        return True
        return False
    def check_spin_time(self, duration):
        elapsed = self.get_clock().now() - self.state_ts
        return elapsed >= Duration(seconds=duration)
    
    def play_audio(self, audio_file):
        os.system(f"mpg123 ~/rat-roulette/ros2_ws/src/rat_roulette_pkg/rat_roulette_pkg/audio/{audio_file}")

def main(args=None):
    print('Hi from rat_roulette_pkg.')
    rclpy.init(args=args)
    rat_roulette = RatRoulette()
    rclpy.spin(rat_roulette)

    rat_roulette.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


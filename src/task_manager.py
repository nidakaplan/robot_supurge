#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pyzbar import pyzbar

# ------------------ STATE TANIMLARI ------------------
INIT = "INIT"
GO_TO_ROOM_ENTRY = "GO_TO_ROOM_ENTRY"
NEXT_ROOM = "NEXT_ROOM"
FINISH = "FINISH"
QR_VERIFY = "QR_VERIFY"
EXECUTE_CLEANING = "EXECUTE_CLEANING"


class TaskManager:
    def __init__(self):
        rospy.init_node("task_manager")

        # move_base action client
        self.client = actionlib.SimpleActionClient(
            "move_base",
            MoveBaseAction
        )

        # Yaml parametreleri
        self.rooms_order = rospy.get_param("/mission/rooms_order")
        self.rooms = rospy.get_param("/rooms")
        self.rooms_dict = {room["name"]: room for room in self.rooms}

        # Navigation ayarları
        self.max_attempts = 2
        self.timeout_sec = 60

        # State bilgileri
        self.state = INIT
        self.current_room_index = 0

        self.bridge = CvBridge()
        self.latest_image = None

        self.image_sub = rospy.Subscriber(
            "/camera/rgb/image_raw",
            Image,
            self.image_callback
        )

        rospy.loginfo("Task Manager baslatildi")

        self.run_state_machine()



    # ------------------ STATE MACHINE ------------------
    def run_state_machine(self):
        rate = rospy.Rate(2)

        while not rospy.is_shutdown():

            if self.state == INIT:
                self.handle_init()

            elif self.state == GO_TO_ROOM_ENTRY:
                self.handle_go_to_room_entry()

            elif self.state == QR_VERIFY:
                self.handle_qr_verify()

            elif self.state == EXECUTE_CLEANING:
                self.handle_execute_cleaning()

            elif self.state == NEXT_ROOM:
                self.handle_next_room()

            elif self.state == FINISH:
                rospy.loginfo("Tum odalar tamamlandi. Gorev bitti.")
                break

            rate.sleep()



    # ------------------ INIT STATE ------------------
    def handle_init(self):
        rospy.loginfo("INIT: move_base bekleniyor...")
        self.client.wait_for_server()
        rospy.loginfo("INIT: move_base hazir")

        self.state = GO_TO_ROOM_ENTRY



    # ------------------ GO_TO_ROOM_ENTRY STATE ------------------
    def handle_go_to_room_entry(self):
        if self.current_room_index >= len(self.rooms_order):
            self.state = FINISH
            return

        room_name = self.rooms_order[self.current_room_index]
        rospy.loginfo(f"GO_TO_ROOM_ENTRY: {room_name} girisine gidiliyor...")

        if room_name not in self.rooms_dict:
            rospy.logwarn("Oda YAML'da bulunamadi, atlaniyor")
            self.state = NEXT_ROOM
            return

        success = self.go_to_room(room_name)

        if success:
            rospy.loginfo(f"{room_name} girisine basariyla ulasildi")
            self.state = QR_VERIFY
        else:
            rospy.logwarn(f"{room_name} girisine ulasilamadi")
            self.state = NEXT_ROOM

        rospy.sleep(0.2)



    # ------------------ QR_VERIFY STATE ------------------
    def handle_qr_verify(self):
        room_name = self.rooms_order[self.current_room_index]
        room = self.rooms_dict[room_name]

        expected_qr = room.get("qr_expected", "")
        rospy.loginfo(f"QR_VERIFY: {room_name}")
        rospy.loginfo(f"Beklenen QR: {expected_qr}")

        start_time = rospy.Time.now()
        timeout = rospy.Duration(5)

        qr_verified = False

        while rospy.Time.now() - start_time < timeout and not rospy.is_shutdown():

            if self.latest_image is None:
                rospy.sleep(0.1)
                continue

            decoded_objects = pyzbar.decode(self.latest_image)


            for obj in decoded_objects:
                decoded_text = obj.data.decode("utf-8")
                rospy.loginfo(f"QR OKUNDU: {decoded_text}")

                if decoded_text == expected_qr:
                    rospy.loginfo("QR DOGRU")
                    qr_verified = True
                    break
                else: 
                    rospy.loginfo(f"QR YANLIS: {decoded_text}")

            if qr_verified:
                self.state = EXECUTE_CLEANING
                break

            rospy.sleep(0.2)

        if not qr_verified:
            rospy.logwarn("QR DOGRULANAMADI (timeout)")
            self.state = NEXT_ROOM


    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="bgr8"
            )
        except Exception as e:
            rospy.logerr("Image donusturulemedi")




    # ------------------ EXECUTE_CLEANING STATE ------------------
    def handle_execute_cleaning(self):
        rospy.loginfo("EXECUTE_CLEANING: Temizlik basladi")

        from sensor_msgs.msg import LaserScan
        from geometry_msgs.msg import Twist
        import math


        timeout_sec = 60  # odada dolaşma süresi
        DESIRED_LEFT = 0.3
        DESIRED_FRONT = 0.7
        FRONT_LIMIT = 0.4
        DESIRED_FRONT_LEFT = DESIRED_FRONT + 0.2
        LEFT_MAX = 3
        Kp = 0.9
        Kd = 2.5
        prev_error = 0.0

        latest_scan = None

        # Subscriber ve Publisher
        def scan_callback(msg):
            nonlocal latest_scan
            latest_scan = msg

        scan_sub = rospy.Subscriber("/scan", LaserScan, scan_callback)
        cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        rospy.sleep(1)
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)

        cleaning_done = False

        while not rospy.is_shutdown():
            if latest_scan is None:
                rate.sleep()
                continue

            ranges = latest_scan.ranges

            def average(data):
                clean_data = [d for d in data if not math.isinf(d) and not math.isnan(d)]
                return sum(clean_data)/len(clean_data) if clean_data else float('inf')

            front = average(ranges[0:20] + ranges[340:360])
            left = average(ranges[80:100])
            front_left = average(ranges[30:60])

            cmd = Twist()

            # Kapı - boşluk kontrolü
            if left > LEFT_MAX or math.isinf(left):
                left = DESIRED_LEFT + 0.2

            
            error = DESIRED_LEFT - left
            derivative = error - prev_error
            steering = Kp*error + Kd*derivative
            steering = max(min(steering, 0.8), -0.8)
            prev_error = error

            
            if left > LEFT_MAX or math.isinf(left):
                cmd.linear.x = 0.2
                cmd.angular.z = steering
            else:

                if front < DESIRED_FRONT:
                    cmd.linear.x = 0.1
                    cmd.angular.z = -0.5
                    if front < FRONT_LIMIT:
                        cmd.linear.x = -0.15
                        cmd.angular.z = 0.0
                elif front_left < DESIRED_FRONT:
                    cmd.linear.x = 0.15
                    cmd.angular.z = -0.5
                elif front_left < (DESIRED_FRONT_LEFT):
                    cmd.linear.x = 0.15
                    cmd.angular.z = 0.5
                else:
                    cmd.linear.x = 0.2
                    cmd.angular.z = steering

            cmd_pub.publish(cmd)

            if rospy.Time.now() - start_time > rospy.Duration(timeout_sec):
                rospy.logwarn(f"{self.rooms_order[self.current_room_index]} odasi dolasilamadi: SKIPPED")
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                cmd_pub.publish(cmd)
                break

            rate.sleep()

        rospy.loginfo(f"EXECUTE_CLEANING: {self.rooms_order[self.current_room_index]} odasi tamamlandi / SUCCESS")
        self.state = NEXT_ROOM




    # ------------------ NEXT_ROOM STATE ------------------
    def handle_next_room(self):
        self.current_room_index += 1
        self.state = GO_TO_ROOM_ENTRY



    # ------------------ NAVIGATION FONKSIYONU ------------------
    def go_to_room(self, room_name):
        room = self.rooms_dict[room_name]
        goal = self.create_goal(room["entry_goal"])

        for attempt in range(1, self.max_attempts + 1):
            rospy.loginfo(f"Hedef: {room_name} – Deneme: {attempt}")

            self.client.send_goal(goal)

            finished = self.client.wait_for_result(
                rospy.Duration(self.timeout_sec)
            )

            if not finished:
                rospy.logwarn("Timeout! Goal iptal ediliyor")
                self.client.cancel_goal()
            else:
                state = self.client.get_state()
                if state == actionlib.GoalStatus.SUCCEEDED:
                    return True
                else:
                    rospy.logwarn(f"move_base basarisiz, state={state}")

            rospy.sleep(1)

        return False
    


    # ------------------ GOAL OLUŞTURMA ------------------
    def create_goal(self, entry_goal):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = entry_goal["x"]
        goal.target_pose.pose.position.y = entry_goal["y"]
        goal.target_pose.pose.position.z = 0.0

        q = quaternion_from_euler(0, 0, entry_goal["yaw"])
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        return goal


if __name__ == "__main__":
    TaskManager()

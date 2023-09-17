import rclpy
from rclpy.node import Node
from custom_msgs.msg import VisualNav
import sys
import threading
import cv2
from datetime import datetime
import math
import redis
import base64
import numpy as np
from playsound import playsound

sys.path.append('/home/nhuengzii/boom-robot/ros2_ws/src/ai_integration')
from personal_tracker import PersonalTracker, TrackerConfig, TrackResult
from personal_tracker.embedder.available_embedder_models import AvailableEmbedderModels
from personal_tracker.metric import MetricType
from personal_tracker.extentions.hand_triger import HandTriggerResult

# control variables
operation: str = 'wait' # follow, wait, notfound
x: int = 0
y: int = 0


# Remote camera streaming
class VideoStreamWidget(object):

    def __init__(self, src: str):
        self.capture = cv2.VideoCapture(src)
        # Start the thread to read frames from the video stream
        self.thread = threading.Thread(target=self.update, args=())
        self.thread.daemon = True
        self.thread.start()
        self.ret = False
        self.frame = None

    def read(self):
        # Return the latest frame
        return self.ret, self.frame

    def update(self):
        # Read the next frame from the stream in a different thread
        while True:
            if self.capture.isOpened():
                (self.ret, self.frame) = self.capture.read()
class RedisCamera():
    def __init__(self):
        self.r = redis.Redis(host='localhost', port=6379, decode_responses=True)
    def decode_base64(self):
        image = self.r.get("image")
        decoded_data = base64.b64decode(image)
        np_data = np.fromstring(decoded_data,np.uint8)
        img = cv2.imdecode(np_data,cv2.IMREAD_UNCHANGED)
        return img

    def read(self):
        img = self.decode_base64()
        img = cv2.resize(img, (1280, 720))
        return True, img

    def release(self):
        pass

class GoalPositionPublisher(Node):

    def __init__(self):
        super().__init__('goal_position_publisher')
        self.publisher_ = self.create_publisher(VisualNav, '/visual_nav', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        ai_thread = threading.Thread(target=self.ai)
        ai_thread.start()
        self.ready = False

    def timer_callback(self):
        global operation, x, y, ready
        msg = VisualNav()
        msg.operation =  operation
        msg.x = x
        msg.y = y
        if operation == 'wait' or self.ready:
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: operation: {operation}, x: {x}, y: {y}\n')
    def command_ros(self, result: TrackResult):
        global operation, x, y
        if not result.success:
            operation = 'notfound'
            x = self.frame_width // 2
            y = self.frame_height // 2
            return
        
        target_bbox = result.target_bbox
        x1, x2 = target_bbox[0], target_bbox[2]
        frame_center_x = self.frame_width // 2
        l, r = frame_center_x - 0.45 * self.frame_width, frame_center_x + 0.45 * self.frame_width
        w = target_bbox[2] - target_bbox[0]
        if x1 >= l and x2 <= r and w >= 0.3 * self.frame_width:
            operation = 'wait'
            return
        operation = 'follow'
        x, y = (target_bbox[0] + target_bbox[2]) // 2, (target_bbox[1] + target_bbox[3]) // 2

        
    
    def ai(self):
        # cap = cv2.VideoCapture(0)
        self.frame_width = 1280 
        self.frame_height = 720
        self.frame_rate = 30
        # cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        # cap.set(cv2.CAP_PROP_FPS, self.frame_rate)
        url = "http://192.168.1.104:8080/video"
        cap = RedisCamera()
            
        config = TrackerConfig().set_embedder_model(AvailableEmbedderModels.OSNET_AIN_X1_0).set_metric_type(MetricType.COSINE_SIMILARITY)
        config = config.set_auto_add_target_features(True, 20).set_sift_history_size(2)
        tracker = PersonalTracker(config)
        hand_trigger = HandTriggerResult()

        def check_hand_break(crop) -> bool:
            
            frame, hand_lms = hand_trigger.process_hands(crop, draw=False)
            is_stop = hand_trigger.find_stop(hand_lms)

            
            print("detect ? :", hand_trigger.is_detect)
            
            global operation
            print(f"Hand trigger, is_stop: {hand_trigger.is_detect}, {is_stop}")
            if operation == 'wait' and is_stop:
                operation = 'follow'
            if is_stop:
                operation = 'wait'
        
        def find_target(cap) -> tuple[int, int, int, int]:
            hand_trigger = HandTriggerResult()
            while True:
                ret, frame = cap.read()
                if not ret:
                    continue
                detect_result = tracker._detector.detect(frame)
                if detect_result is None:
                    cv2.imshow("result", frame)
                    continue

                for target in detect_result.bboxes:
                    croped = frame[target[1]:target[3], target[0]:target[2]]
                    hand_trigger.process_hands(croped, draw=True)
                    if hand_trigger.is_detect:
                        global deteted
                        deteted = True
                        return target
                cv2.waitKey(1)
                cv2.imshow("result", frame)
        def get_target(cap, target_bbox, second=3):
            cx, cy = (target_bbox[0] + target_bbox[2]) // 2, (target_bbox[1] + target_bbox[3]) // 2
            targets = []
            start_time = datetime.now()
            while (datetime.now() - start_time).seconds < second:
                ret, frame = cap.read()
                if not ret:
                    continue

                detect_result = tracker._detector.detect(frame)
                if detect_result is None:
                    continue
                ranks = tracker._tracker._kalman_ranking((cx, cy), detect_result.bboxes)
                target_bbox = detect_result.bboxes[ranks[0]]
                targets.append((frame, target_bbox))
                cv2.waitKey(200)
            return targets
            

        target_bbox = find_target(cap)
        targets = get_target(cap, target_bbox)
        playsound("/home/nhuengzii/boom-robot/start_tracking.mp3")
        targets = tracker.get_repetitive_target_from_camera(cap, 3)
        for target in targets:
            tracker.add_target_features(target[0], target[1])
        print(f"Start tracking with {len(targets)} num targets")
        self.ready = True
        while True:
            ret, raw_frame = cap.read()
            frame = raw_frame.copy()
            if not ret:
                print("Can't read frame")
                continue
            result = tracker.track(frame, draw_result=True)
            if result.success:
                target_bbox = result.target_bbox
                croped = frame[target_bbox[1]:target_bbox[3], target_bbox[0]:target_bbox[2]]
                # check_hand_break(croped)
            tagets_show = tracker._tracker.show_target_images()
            cv2.imshow("result", frame)
            cv2.imshow("targets", tagets_show)
            # if operation == "wait":
            #     if cv2.waitKey(1) == 's':
            #         operation = 'follow'
            self.command_ros(result)
            if cv2.waitKey(2) == ord("q"):
                break
        cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    goal_position_publisher = GoalPositionPublisher()
    rclpy.spin(goal_position_publisher)
    goal_position_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

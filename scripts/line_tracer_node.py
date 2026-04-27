#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
line_tracer_node.py  v9
────────────────────────────────────────────────────────
변경 이력:
  v1  - 기본 단일 ROI + 고정 임계값
  v2  - 비선형 PID (에러 크기별 Kp 자동 증가)
  v3  - 적응형 임계값 (흰 바닥 노이즈 제거)
  v4  - 듀얼 ROI Lookahead
  v5  - 검은 트랙 추적 시도
  v6  - 흰선 + 컨투어 폭 필터로 바닥 블롭 제거
  v7  - 에러 변화율 기반 급커브 선제 감속, 한쪽 선 강한 회전
  v8  - 버그 수정: error_rate 계산 순서, 직진 speed_scale 상한 낮춤
  v9  - Last Known Error 유지 로직:
        선이 사라졌을 때 무조건 고정 회전 대신
        직전 에러로 PID 계속 실행 (최대 lost_max_frames 프레임)
        프레임 초과 시에만 정지
────────────────────────────────────────────────────────
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError


class PIDController:
    """비선형 PID 제어기 — 에러 크기에 따라 Kp 자동 증가"""

    def __init__(self, kp, ki, kd, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error):
        self.integral += error * self.dt
        self.integral = np.clip(self.integral, -300.0, 300.0)
        derivative = (error - self.prev_error) / self.dt

        abs_err = abs(error)
        if abs_err > 60:
            dynamic_kp = self.kp * 4.0
        elif abs_err > 30:
            dynamic_kp = self.kp * 2.5
        else:
            dynamic_kp = self.kp

        output = -(dynamic_kp * error
                   + self.ki * self.integral
                   + self.kd * derivative)
        self.prev_error = error
        return float(np.clip(output, -2.0, 2.0))

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0


class LineTracer:
    def __init__(self):
        rospy.init_node('line_tracer_node', anonymous=False)

        # ── 파라미터 로드 ──────────────────────────────────────────────
        self.linear_speed         = rospy.get_param('~linear_speed',         0.06)
        self.kp                   = rospy.get_param('~kp',                   0.004)
        self.ki                   = rospy.get_param('~ki',                   0.0)
        self.kd                   = rospy.get_param('~kd',                   0.005)
        self.dt                   = rospy.get_param('~dt',                   0.05)
        self.camera_topic         = rospy.get_param('~camera_topic',         '/camera/image')
        self.show_debug_window    = rospy.get_param('~show_debug_window',    True)
        self.flip_image           = rospy.get_param('~flip_image',           False)

        # 흰 선 검출
        self.white_threshold      = rospy.get_param('~white_threshold',      170)
        self.min_line_area        = rospy.get_param('~min_line_area',        80)
        self.max_blob_width_ratio = rospy.get_param('~max_blob_width_ratio', 0.35)

        # 듀얼 ROI
        self.far_start_ratio      = rospy.get_param('~far_start_ratio',      0.75)
        self.far_end_ratio        = rospy.get_param('~far_end_ratio',        0.85)
        self.near_start_ratio     = rospy.get_param('~near_start_ratio',     0.65)
        self.far_weight           = rospy.get_param('~far_weight',           0.0)
        self.near_weight          = rospy.get_param('~near_weight',          1.0)

        # 급커브 감속 임계값
        self.curve_rate_threshold = rospy.get_param('~curve_rate_threshold', 150.0)

        # [v9] Last Known Error 유지 최대 프레임 수 (launch에서 조정 가능)
        self.lost_max_frames      = rospy.get_param('~lost_max_frames',      8)

        # ── PID + 상태 저장 ─────────────────────────────────────────────
        self.pid    = PIDController(self.kp, self.ki, self.kd, self.dt)
        self.bridge = CvBridge()

        # [v9] 마지막으로 인식한 에러와 소실 프레임 카운터
        self.last_known_error = 0.0
        self.lost_frames      = 0

        # ── ROS 통신 ─────────────────────────────────────────────────────
        self.cmd_pub   = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.debug_pub = rospy.Publisher('/line_tracer/debug_image', Image, queue_size=1)
        self.image_sub = rospy.Subscriber(
            self.camera_topic, Image, self.image_callback,
            queue_size=1, buff_size=2**24)

        self.twist = Twist()
        rospy.on_shutdown(self.shutdown_hook)
        rospy.loginfo("[LineTracer] v9 시작 | Last Known Error 유지 모드")

    # ────────────────────────────────────────────────────────────────────
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            rospy.logerr("[LineTracer] CvBridge 오류: %s", e)
            return
        if self.flip_image:
            frame = cv2.flip(frame, 1)
        self.process_frame(frame)

    # ────────────────────────────────────────────────────────────────────
    def process_frame(self, frame):
        h, w = frame.shape[:2]

        far_start  = int(h * self.far_start_ratio)
        far_end    = int(h * self.far_end_ratio)
        near_start = int(h * self.near_start_ratio)

        far_roi  = frame[far_start:far_end, 0:w]
        near_roi = frame[near_start:h,      0:w]

        far_bin  = self._binarize_white(far_roi)
        near_bin = self._binarize_white(near_roi)

        mid_w = w // 2

        far_left_cx   = self._get_line_cx(far_bin[:,  :mid_w], offset_x=0,     roi_w=mid_w)
        far_right_cx  = self._get_line_cx(far_bin[:,  mid_w:], offset_x=mid_w, roi_w=mid_w)
        near_left_cx  = self._get_line_cx(near_bin[:, :mid_w], offset_x=0,     roi_w=mid_w)
        near_right_cx = self._get_line_cx(near_bin[:, mid_w:], offset_x=mid_w, roi_w=mid_w)

        # ── 에러 계산 ─────────────────────────────────────────────────
        far_error  = None
        near_error = None

        if far_left_cx is not None and far_right_cx is not None:
            far_error = (far_left_cx + far_right_cx) / 2.0 - w / 2.0
        if near_left_cx is not None and near_right_cx is not None:
            near_error = (near_left_cx + near_right_cx) / 2.0 - w / 2.0

        # ── 블렌딩 ────────────────────────────────────────────────────
        if far_error is not None and near_error is not None:
            blended_error = self.far_weight * far_error + self.near_weight * near_error
            mode = "DUAL"
        elif near_error is not None:
            blended_error = near_error
            mode = "NEAR"
        elif far_error is not None:
            blended_error = far_error
            mode = "FAR"
        else:
            blended_error = None
            mode = "NONE"

        # ── 디버그 프레임 ─────────────────────────────────────────────
        debug = frame.copy()
        cv2.rectangle(debug, (0, far_start),  (w-1, far_end-1), (255, 165, 0), 1)
        cv2.rectangle(debug, (0, near_start), (w-1, h-1),       (0, 255, 255), 2)
        cv2.line(debug, (mid_w, 0), (mid_w, h), (180, 180, 180), 1)
        cv2.line(debug, (w//2,  0), (w//2,  h), (255, 0,   255), 1)

        far_cy  = (far_start + far_end) // 2
        near_cy = (near_start + h) // 2

        # ────────────────────────────────────────────────────────────────────
        # 제어 — [v9] 선 소실 시 Last Known Error 로직 적용
        # ────────────────────────────────────────────────────────────────────
        if blended_error is not None:
            # 선 정상 인식 → 마지막 에러 갱신, 소실 프레임 리셋
            self.last_known_error = blended_error
            self.lost_frames = 0

            prev_err_snapshot = self.pid.prev_error
            angular_z = self.pid.compute(blended_error)
            error_rate = abs(blended_error - prev_err_snapshot) / self.dt

            if error_rate > self.curve_rate_threshold:
                speed_scale = 0.15
            elif abs(blended_error) > 50:
                speed_scale = 0.20
            elif abs(blended_error) > 30:
                speed_scale = 0.35
            else:
                speed_scale = max(0.15, 1.0 - abs(angular_z) * 1.5)

            dynamic_speed = self.linear_speed * speed_scale
            self._publish_cmd(dynamic_speed, angular_z)

            if far_left_cx   is not None: cv2.circle(debug, (far_left_cx,   far_cy),  7, (0,180,0),   -1)
            if far_right_cx  is not None: cv2.circle(debug, (far_right_cx,  far_cy),  7, (0,0,180),   -1)
            if near_left_cx  is not None: cv2.circle(debug, (near_left_cx,  near_cy), 9, (0,255,0),   -1)
            if near_right_cx is not None: cv2.circle(debug, (near_right_cx, near_cy), 9, (0,0,255),   -1)

            target_x = int(w / 2.0 + blended_error)
            cv2.circle(debug, (target_x, near_cy), 11, (0,255,255), -1)
            cv2.circle(debug, (w//2,     near_cy), 11, (255,0,255),  2)
            cv2.line(debug, (w//2, near_cy), (target_x, near_cy), (0,165,255), 2)
            cv2.putText(debug,
                        f"{mode} err={blended_error:+.0f} ang={angular_z:+.3f} spd={dynamic_speed:.3f}",
                        (5, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0,255,0), 2)

        else:
            # ── [v9] 선 없음: Last Known Error로 코스 유지 ────────────────
            self.lost_frames += 1

            if self.lost_frames <= self.lost_max_frames:
                # 직전 에러 방향으로 PID 계속 실행
                angular_z    = self.pid.compute(self.last_known_error)
                dynamic_speed = self.linear_speed * 0.15  # 느린 속도로 커브 진행
                self._publish_cmd(dynamic_speed, angular_z)

                remaining = self.lost_max_frames - self.lost_frames
                cv2.putText(debug,
                            f"LOST [{remaining}] last_err={self.last_known_error:+.0f} ang={angular_z:+.3f}",
                            (5, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0,165,255), 2)
            else:
                # 프레임 초과 → 전수정지
                self.pid.reset()
                self._publish_cmd(0.0, 0.0)
                cv2.putText(debug, "NO LINE  -- STOP",
                            (5, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,0,255), 2)

        # ── 미니맵 ────────────────────────────────────────────────────
        mini_h = max(1, (h - near_start) // 3)
        mini_w = max(1, w // 3)
        mini = cv2.resize(cv2.cvtColor(near_bin, cv2.COLOR_GRAY2BGR), (mini_w, mini_h))
        debug[h-mini_h:h, w-mini_w:w] = mini
        cv2.rectangle(debug, (w-mini_w, h-mini_h), (w-1, h-1), (0,255,255), 1)

        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))
        except CvBridgeError:
            pass

        if self.show_debug_window:
            cv2.imshow("LineTracer Debug", debug)
            cv2.waitKey(1)

    # ────────────────────────────────────────────────────────────────────
    def _binarize_white(self, roi):
        gray  = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur  = cv2.GaussianBlur(gray, (5, 5), 0)
        _, binary = cv2.threshold(blur, self.white_threshold, 255, cv2.THRESH_BINARY)
        kernel = np.ones((3, 3), np.uint8)
        return cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

    def _get_line_cx(self, binary_roi, offset_x, roi_w):
        contours, _ = cv2.findContours(binary_roi, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        best_cx   = None
        best_area = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_line_area:
                continue
            x, y, bw, bh = cv2.boundingRect(cnt)
            if bw > roi_w * self.max_blob_width_ratio:
                continue
            if area > best_area:
                best_area = area
                M = cv2.moments(cnt)
                if M['m00'] > 0:
                    best_cx = int(M['m10'] / M['m00']) + offset_x
        return best_cx

    def _publish_cmd(self, linear, angular):
        self.twist.linear.x  = linear
        self.twist.angular.z = angular
        self.cmd_pub.publish(self.twist)

    def shutdown_hook(self):
        rospy.loginfo("[LineTracer] 종료")
        self._publish_cmd(0.0, 0.0)
        cv2.destroyAllWindows()


# ════════════════════════════════════════════════════════════════════════
if __name__ == '__main__':
    try:
        node = LineTracer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

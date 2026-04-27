#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
line_tracer_node.py  v10
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
  v9  - Last Known Error 유지 로직 (선 소실 시 최대 N프레임 PID 유지)
  v10 - 2단 ROI 전면 개편:
        drive_roi(아래 주행 결정) + predict_roi(위 선행 예측)
        최종 조향 = drive * 0.75 + predict * 0.25
        주황 박스 = 예측 ROI, 빨강 박스 = 주행 ROI
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
        self.integral   = 0.0

    def compute(self, error):
        self.integral += error * self.dt
        self.integral  = np.clip(self.integral, -300.0, 300.0)
        derivative     = (error - self.prev_error) / self.dt

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
        self.integral   = 0.0


class LineTracer:
    def __init__(self):
        rospy.init_node('line_tracer_node', anonymous=False)

        # ── 기본 파라미터 ─────────────────────────────────────────────
        self.linear_speed         = rospy.get_param('~linear_speed',         0.05)
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

        # ── [v10] 2단 ROI ─────────────────────────────────────────────
        # predict_roi: 주행 ROI 바로 위 → 커브 선행 예측
        self.predict_start_ratio  = rospy.get_param('~predict_start_ratio',  0.42)
        self.predict_end_ratio    = rospy.get_param('~predict_end_ratio',    0.56)
        # drive_roi: 실제 주행 조향 결정
        self.drive_start_ratio    = rospy.get_param('~drive_start_ratio',    0.58)
        self.drive_end_ratio      = rospy.get_param('~drive_end_ratio',      0.82)
        # 블렌딩 가중치
        self.drive_weight         = rospy.get_param('~drive_weight',         0.75)
        self.predict_weight       = rospy.get_param('~predict_weight',       0.25)

        # 커브 감속 임계값
        self.curve_rate_threshold = rospy.get_param('~curve_rate_threshold', 120.0)

        # Last Known Error 유지 최대 프레임
        self.lost_max_frames      = rospy.get_param('~lost_max_frames',      8)

        # ── PID + 상태 ────────────────────────────────────────────────
        self.pid              = PIDController(self.kp, self.ki, self.kd, self.dt)
        self.bridge           = CvBridge()
        self.last_known_error = 0.0
        self.lost_frames      = 0

        # ── ROS ───────────────────────────────────────────────────────
        self.cmd_pub   = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.debug_pub = rospy.Publisher('/line_tracer/debug_image', Image, queue_size=1)
        self.image_sub = rospy.Subscriber(
            self.camera_topic, Image, self.image_callback,
            queue_size=1, buff_size=2**24)

        self.twist = Twist()
        rospy.on_shutdown(self.shutdown_hook)
        rospy.loginfo("[LineTracer] v10 시작 | 2단 ROI (예측+주행)")

    # ─────────────────────────────────────────────────────────────────
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            rospy.logerr("[LineTracer] CvBridge 오류: %s", e)
            return
        if self.flip_image:
            frame = cv2.flip(frame, 1)
        self.process_frame(frame)

    # ─────────────────────────────────────────────────────────────────
    def process_frame(self, frame):
        h, w = frame.shape[:2]

        # ── ROI 슬라이스 ──────────────────────────────────────────────
        ps = int(h * self.predict_start_ratio)
        pe = int(h * self.predict_end_ratio)
        ds = int(h * self.drive_start_ratio)
        de = int(h * self.drive_end_ratio)

        predict_roi = frame[ps:pe, 0:w]
        drive_roi   = frame[ds:de, 0:w]

        predict_bin = self._binarize_white(predict_roi)
        drive_bin   = self._binarize_white(drive_roi)

        mid_w = w // 2

        # 예측 ROI 선 검출
        pred_L = self._get_line_cx(predict_bin[:, :mid_w], offset_x=0,     roi_w=mid_w)
        pred_R = self._get_line_cx(predict_bin[:, mid_w:], offset_x=mid_w, roi_w=mid_w)
        # 주행 ROI 선 검출
        drv_L  = self._get_line_cx(drive_bin[:,  :mid_w], offset_x=0,     roi_w=mid_w)
        drv_R  = self._get_line_cx(drive_bin[:,  mid_w:], offset_x=mid_w, roi_w=mid_w)

        # ── 에러 계산 ─────────────────────────────────────────────────
        predict_error = None
        drive_error   = None

        if pred_L is not None and pred_R is not None:
            predict_error = (pred_L + pred_R) / 2.0 - w / 2.0
        if drv_L  is not None and drv_R  is not None:
            drive_error   = (drv_L  + drv_R)  / 2.0 - w / 2.0

        # ── 블렌딩 ────────────────────────────────────────────────────
        if drive_error is not None and predict_error is not None:
            blended_error = self.drive_weight * drive_error + self.predict_weight * predict_error
            mode = "DRV+PRD"
        elif drive_error is not None:
            blended_error = drive_error
            mode = "DRIVE"
        elif predict_error is not None:
            blended_error = predict_error
            mode = "PRED"
        else:
            blended_error = None
            mode = "NONE"

        # ── 디버그 프레임 ─────────────────────────────────────────────
        debug = frame.copy()
        # 주황 박스 = 예측 ROI
        cv2.rectangle(debug, (0, ps), (w-1, pe-1), (0, 165, 255), 2)
        cv2.putText(debug, "PRED", (5, ps + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 165, 255), 1)
        # 빨강 박스 = 주행 ROI
        cv2.rectangle(debug, (0, ds), (w-1, de-1), (0, 0, 255), 2)
        cv2.putText(debug, "DRIVE", (5, ds + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 0, 255), 1)
        # 중앙선
        cv2.line(debug, (w//2, 0), (w//2, h), (200, 200, 200), 1)

        pred_cy = (ps + pe) // 2
        drv_cy  = (ds + de) // 2

        # ── 제어 ──────────────────────────────────────────────────────
        if blended_error is not None:
            self.last_known_error = blended_error
            self.lost_frames      = 0

            prev_snap  = self.pid.prev_error
            angular_z  = self.pid.compute(blended_error)
            error_rate = abs(blended_error - prev_snap) / self.dt

            # 속도 스케일 결정
            if mode == "PRED":
                speed_scale = 0.10          # 아래 선 없고 위만 보임 → 매우 천천히
            elif error_rate > self.curve_rate_threshold:
                speed_scale = 0.15          # 급커브 감속
            elif abs(blended_error) > 50:
                speed_scale = 0.20
            elif abs(blended_error) > 30:
                speed_scale = 0.35
            else:
                speed_scale = max(0.20, 1.0 - abs(angular_z) * 1.5)

            dynamic_speed = self.linear_speed * speed_scale
            self._publish_cmd(dynamic_speed, angular_z)

            # 검출 점 표시
            if pred_L is not None: cv2.circle(debug, (pred_L, pred_cy), 6, (0,200,0),  -1)
            if pred_R is not None: cv2.circle(debug, (pred_R, pred_cy), 6, (200,0,0),  -1)
            if drv_L  is not None: cv2.circle(debug, (drv_L,  drv_cy),  9, (0,255,0),  -1)
            if drv_R  is not None: cv2.circle(debug, (drv_R,  drv_cy),  9, (0,0,255),  -1)

            # 목표점 & 중심점 표시
            target_x = int(w / 2.0 + blended_error)
            cv2.circle(debug, (target_x, drv_cy), 11, (0,255,255), -1)
            cv2.circle(debug, (w//2,     drv_cy), 11, (255,0,255),  2)
            cv2.line(debug, (w//2, drv_cy), (target_x, drv_cy), (0,165,255), 2)

            cv2.putText(debug,
                        f"{mode} err={blended_error:+.0f} ang={angular_z:+.3f} spd={dynamic_speed:.3f}",
                        (5, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0,255,0), 2)

        else:
            # ── Last Known Error 유지 ────────────────────────────────
            self.lost_frames += 1

            if self.lost_frames <= self.lost_max_frames:
                angular_z     = self.pid.compute(self.last_known_error)
                dynamic_speed = self.linear_speed * 0.12
                self._publish_cmd(dynamic_speed, angular_z)

                remaining = self.lost_max_frames - self.lost_frames
                cv2.putText(debug,
                            f"LOST [{remaining}] last_err={self.last_known_error:+.0f} ang={angular_z:+.3f}",
                            (5, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0,165,255), 2)
            else:
                self.pid.reset()
                self._publish_cmd(0.0, 0.0)
                cv2.putText(debug, "NO LINE  -- STOP",
                            (5, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,0,255), 2)

        # ── 미니맵 (주행 ROI 이진화) ──────────────────────────────────
        mini_h = max(1, (de - ds) // 3)
        mini_w = max(1, w // 3)
        mini   = cv2.resize(cv2.cvtColor(drive_bin, cv2.COLOR_GRAY2BGR), (mini_w, mini_h))
        debug[h-mini_h:h, w-mini_w:w] = mini
        cv2.rectangle(debug, (w-mini_w, h-mini_h), (w-1, h-1), (0,255,255), 1)

        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))
        except CvBridgeError:
            pass

        if self.show_debug_window:
            cv2.imshow("LineTracer Debug", debug)
            cv2.waitKey(1)

    # ─────────────────────────────────────────────────────────────────
    def _binarize_white(self, roi):
        gray   = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur   = cv2.GaussianBlur(gray, (5, 5), 0)
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

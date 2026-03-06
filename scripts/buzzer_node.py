#!/usr/bin/env python3
"""GPIO 부저 노드.

/robot/buzzer (std_msgs/Bool) 를 구독해서
True 수신 시 라즈베리파이 GPIO 핀으로 부저를 패턴에 맞게 울린다.

배선:
    GPIO_pin ──[100Ω]── 부저(+)
    GND      ────────── 부저(-)
    (수동 부저 사용 시 PWM 필요, 능동 부저는 PWM 선택 사항)

ROS2 parameters:
    gpio_pin     (int)    BCM 핀 번호, default: 17
    frequency    (int)    PWM 주파수(Hz), 수동 부저용, default: 2000
    pattern      (string) "short" | "double" | "long", default: "short"
"""

import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

try:
    import RPi.GPIO as GPIO
    _HAS_GPIO = True
except ImportError:
    _HAS_GPIO = False


# 패턴 정의: (on_sec, off_sec) 리스트
PATTERNS = {
    'short':  [(0.15, 0.0)],
    'double': [(0.15, 0.1), (0.15, 0.0)],
    'long':   [(0.8, 0.0)],
}


class BuzzerNode(Node):

    def __init__(self):
        super().__init__('buzzer_node')

        self.declare_parameter('gpio_pin', 17)
        self.declare_parameter('frequency', 2000)
        self.declare_parameter('pattern', 'short')

        self._pin = self.get_parameter('gpio_pin').get_parameter_value().integer_value
        self._freq = self.get_parameter('frequency').get_parameter_value().integer_value
        self._pattern_name = (self.get_parameter('pattern')
                               .get_parameter_value().string_value)

        self._pwm = None
        self._lock = threading.Lock()
        self._busy = False

        if _HAS_GPIO:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self._pin, GPIO.OUT, initial=GPIO.LOW)
            self._pwm = GPIO.PWM(self._pin, self._freq)
            self.get_logger().info(
                f'GPIO 초기화 완료 — pin={self._pin}  freq={self._freq}Hz')
        else:
            self.get_logger().warn(
                'RPi.GPIO 없음 — 시뮬레이션 모드로 실행 (부저 출력 없음)')

        self._sub = self.create_subscription(
            Bool, '/robot/buzzer', self._buzzer_cb, 10)

        self.get_logger().info(f'BuzzerNode 준비 — 패턴: {self._pattern_name}')

    def _buzzer_cb(self, msg: Bool):
        if not msg.data:
            return
        if self._busy:
            return
        pattern = PATTERNS.get(self._pattern_name, PATTERNS['short'])
        t = threading.Thread(target=self._play, args=(pattern,), daemon=True)
        t.start()

    def _play(self, pattern):
        with self._lock:
            self._busy = True
            try:
                for on_sec, off_sec in pattern:
                    self._beep_on()
                    time.sleep(on_sec)
                    self._beep_off()
                    if off_sec > 0:
                        time.sleep(off_sec)
            finally:
                self._busy = False

    def _beep_on(self):
        if _HAS_GPIO and self._pwm:
            self._pwm.start(50)
        else:
            self.get_logger().info('[sim] 부저 ON')

    def _beep_off(self):
        if _HAS_GPIO and self._pwm:
            self._pwm.stop()
        else:
            self.get_logger().info('[sim] 부저 OFF')

    def destroy_node(self):
        if _HAS_GPIO:
            if self._pwm:
                self._pwm.stop()
            GPIO.cleanup(self._pin)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BuzzerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

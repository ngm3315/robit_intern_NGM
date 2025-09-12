# turtlesim을 WASD와 t/r/c로 조작해 도형을 그리는 rclpy 노드
# - /turtle1/cmd_vel 퍼블리시
# - /turtle1/set_pen 서비스로 펜 색/굵기 변경
# - t: 삼각형(빨강, 굵기3), r: 사각형(초록, 굵기4), c: 원(파랑, 굵기2), q: 종료

import sys, time, termios, tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen

def getch() -> str:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

class TurtleSteering(Node):
    def __init__(self):
        super().__init__('turtle_steering_py')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pen_cli = self.create_client(SetPen, '/turtle1/set_pen')

    # lin: m/s, ang: rad/s, duration: s
    def move(self, lin: float, ang: float, duration: float = 1.0, hz: float = 10.0):
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        dt = 1.0 / hz
        steps = int(duration * hz)
        for _ in range(steps):
            self.pub.publish(msg)
            time.sleep(dt)
        self.stop()

    def stop(self):
        self.pub.publish(Twist())

    def set_pen(self, r: int, g: int, b: int, width: int, off: bool = False):
        while not self.pen_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting /turtle1/set_pen')
        req = SetPen.Request()
        req.r, req.g, req.b, req.width, req.off = r, g, b, width, off
        fut = self.pen_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)

    def draw_triangle(self):
        self.set_pen(255, 0, 0, 3)
        for _ in range(3):
            self.move(2.0, 0.0, 1.5)
            self.move(0.0, 2.1, 1.0)  # ≈120°

    def draw_square(self):
        self.set_pen(0, 255, 0, 4)
        for _ in range(4):
            self.move(2.0, 0.0, 1.5)
            self.move(0.0, 1.57, 1.0)  # ≈90°

    def draw_circle(self):
        self.set_pen(0, 0, 255, 2)
        self.move(2.0, 1.0, 6.5)

    def run(self):
        print('WASD 이동, t=삼각형, r=사각형, c=원, q=종료')
        while rclpy.ok():
            k = getch()
            if k == 'q': break
            elif k == 'w': self.move(2.0, 0.0)
            elif k == 's': self.move(-2.0, 0.0)
            elif k == 'a': self.move(0.0, 2.0)
            elif k == 'd': self.move(0.0, -2.0)
            elif k == 't': self.draw_triangle()
            elif k == 'r': self.draw_square()
            elif k == 'c': self.draw_circle()

def main(args=None):
    rclpy.init(args=args)
    node = TurtleSteering()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

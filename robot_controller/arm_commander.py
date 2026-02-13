# robot_controller/robot_controller/arm_commander.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
import time

class ArmCommander(Node):
    def __init__(self):
        super().__init__('arm_commander')
        self.get_logger().info('ArmCommander started')

        # Joint names must match URDF joints
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        # Limits (from URDF)
        self.joint_limits = {
            'joint1': (-3.14, 3.14),
            'joint2': (-1.57, 1.57),
            'joint3': (-1.57, 1.57),
            'joint4': (-1.57, 1.57)
        }

        # current positions initialized at mid-point
        self.positions = {}
        for j in self.joint_names:
            lo, hi = self.joint_limits[j]
            self.positions[j] = 0.5 * (lo + hi)

        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.sub = self.create_subscription(Int32, 'finger_count', self.finger_cb, 10)

        # Which joint is currently toggling (1..4) or None; 5 means move all to min.
        self.active_mode = None
        self.move_all_to_min = False

        # For toggling motion: per joint direction (1 or -1) and speed rad/s
        self.directions = {j: 1 for j in self.joint_names}
        self.speed = 1.0  # rad/s — feel free to tune
        self.last_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec*1e-9

        # timer for publishing at 30 Hz
        self.timer = self.create_timer(1.0/30.0, self.timer_cb)

    def finger_cb(self, msg):
        n = int(msg.data)
        self.get_logger().info(f'Finger count received: {n}')
        if n in (1,2,3,4):
            self.active_mode = n  # toggle joint n
            self.move_all_to_min = False
        elif n == 5:
            self.active_mode = None
            self.move_all_to_min = True
        else:
            # 0 or others: stop motion
            self.active_mode = None
            self.move_all_to_min = False

    def timer_cb(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        # compute dt since last update
        try:
            dt = now - self._last_time
        except AttributeError:
            dt = 0.033
        self._last_time = now

        if self.move_all_to_min:
            # Move all joints smoothly to their min limit
            done = True
            for j in self.joint_names:
                lo, hi = self.joint_limits[j]
                current = self.positions[j]
                # simple linear step toward lo
                step = self.speed * dt
                if current > lo + 1e-3:
                    self.positions[j] = max(lo, current - step)
                    done = False
            # keep publishing until done
        elif self.active_mode in (1,2,3,4):
            idx = self.active_mode - 1
            joint = self.joint_names[idx]
            lo, hi = self.joint_limits[joint]
            cur = self.positions[joint]
            dirn = self.directions[joint]
            # update position
            step = dirn * self.speed * dt
            new = cur + step
            # if exceed limits, reverse direction and clamp
            if new > hi:
                new = hi
                self.directions[joint] = -1
            elif new < lo:
                new = lo
                self.directions[joint] = 1
            self.positions[joint] = new
            # other joints hold their current positions
        else:
            # inactive: hold positions (no motion)
            pass

        # compose JointState message
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = [self.positions[j] for j in self.joint_names]
        # velocities and efforts left empty
        self.publisher_.publish(js)

def main(args=None):
    rclpy.init(args=args)
    node = ArmCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

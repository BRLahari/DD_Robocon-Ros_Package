import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import curses
import time

class Controller(Node):
    def __init__(self):
        super().__init__("Controller")
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.linear_speed = 0.5
        self.angular_speed = 1.0
        self.active_keys = set()  # Track pressed keys
        self.running = True
        self.last_published_twist = None  # Track last sent message
        self.get_logger().info("Controller initialized")

    def update_velocity(self):
        """Updates and publishes velocity based on active keys only when needed."""
        if not self.active_keys:
            return  # No movement, don't publish anything

        twist = Twist()

        # Linear movement
        if ord('w') in self.active_keys:
            twist.linear.x += self.linear_speed
        if ord('x') in self.active_keys:
            twist.linear.x -= self.linear_speed
        if ord('a') in self.active_keys:
            twist.linear.y += self.linear_speed
        if ord('d') in self.active_keys:
            twist.linear.y -= self.linear_speed

        # Rotation
        if ord('q') in self.active_keys:
            twist.angular.z += self.angular_speed
        if ord('e') in self.active_keys:
            twist.angular.z -= self.angular_speed

        # Only publish if the twist is different from the last published one
        if self.last_published_twist is None or twist != self.last_published_twist:
            self.publisher.publish(twist)
            self.get_logger().info(f"Velocity: x={twist.linear.x}, y={twist.linear.y}, z={twist.angular.z}")
            self.last_published_twist = twist  # Update last published twist

    def stop_robot(self):
        """Immediately stops the robot by publishing zero velocity only once."""
        self.active_keys.clear()
        twist = Twist()  # Zero velocity

        # Only publish if last published message is not already zero
        if self.last_published_twist is None or self.last_published_twist != twist:
            self.publisher.publish(twist)
            self.get_logger().info("Robot stopped")
            self.last_published_twist = twist  # Update last published twist

    def run(self):
        curses.wrapper(self.keyboard)

    def keyboard(self, stdscr):
        stdscr.clear()
        stdscr.addstr(0, 0, "Use WASDQE to move, S to stop, Z to quit")
        stdscr.refresh()
        stdscr.nodelay(True)  # Make getch non-blocking

        while rclpy.ok() and self.running:
            key = stdscr.getch()

            if key != -1:
                if key == ord('z'):  # Quit
                    self.running = False
                    break
                elif key == ord('s'):  # Stop
                    self.stop_robot()
                else:
                    self.active_keys.add(key)

            # Check if any keys were released
            if key == -1:
                time.sleep(0.01)  # Give some time to detect key releases
                keys_to_remove = set(self.active_keys)
                for k in keys_to_remove:
                    if stdscr.getch() == -1:
                        self.active_keys.remove(k)

            self.update_velocity()
            time.sleep(0.1)  # Small delay to prevent excessive publishing

def main(args=None):
    rclpy.init(args=args)
    controller = Controller()

    try:
        controller.run()
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()


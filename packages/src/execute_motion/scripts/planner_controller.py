#! /usr/bin/env python3
import rclpy
from execute_motion.planner_controller_interface import PlannerControllerInterface
from rclpy.executors import MultiThreadedExecutor

# Start point of our controller

def main(args=None):

    rclpy.init(args=args)
    node = PlannerControllerInterface()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except rclpy.executors.ExternalShutdownException:
        node.get_logger().info('Program stopped cleanly')
    except KeyboardInterrupt:
        node.get_logger().info('Program stopped cleanly')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

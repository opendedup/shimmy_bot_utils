import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor  # Use for efficient node management
from std_msgs.msg import String
import json

class NodeChecker(Node):
    def __init__(self,namespace="/shimmy_bot"):
        super().__init__('node_checker')
        self.declare_parameter('target_nodes', ['node_checker'])
        self.target_nodes = self.get_parameter('target_nodes').value
        self.publisher_ = self.create_publisher(String, f'{namespace}/node_status', 10)
        self.timer = self.create_timer(1.0, self.check_nodes)  # Check every 1 second

    def check_nodes(self):
        node_list = self.get_node_names()
        node_status = {}

        for target_node in self.target_nodes:
            if target_node in node_list:
                node_status[target_node] = True
            else:
                node_status[target_node] = False

        msg = String()
        msg.data = json.dumps(node_status)
        self.publisher_.publish(msg)
        #self.get_logger().info(f'Node status: {node_status}')

def main(args=None):
    rclpy.init(args=args)
    node_checker = NodeChecker()
    executor = MultiThreadedExecutor()
    executor.add_node(node_checker)
    executor.spin()
    executor.shutdown()
    node_checker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

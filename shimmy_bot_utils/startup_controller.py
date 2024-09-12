import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from chat_interfaces.msg import LedColor, PaintLedColor, Chat
import json
import time
import math

class StartupLEDController(Node):
    def __init__(self,namespace="/shimmy_bot"):
        super().__init__('startup_led_controller')
        self.declare_parameter('target_nodes', ['m_service','voice_emb','image_cap_service','asr_listener'])
        self.target_nodes = self.get_parameter('target_nodes').value
        self.led_publisher_ = self.create_publisher(PaintLedColor, f'{namespace}/ledpaint', 10)
        self.chat_publisher_ = self.create_publisher(Chat, f'{namespace}/asr', 10)
        self.node_status_subscription = self.create_subscription(
            String, 
            f'{namespace}/node_status', 
            self.node_status_callback, 
            10)
        self.m_service_started = False
        self.all_nodes_started = False
        self.timer = self.create_timer(0.5, self.flash_led)

    def node_status_callback(self, msg):
        node_status = json.loads(msg.data)
        if 'm_service' in node_status and node_status['m_service'] is True:
            self.m_service_started = True

            # Check if ALL target nodes are running (only after m_service starts)
            if all(target_node in node_status and node_status[target_node] is True 
                for target_node in self.target_nodes):
                if not self.all_nodes_started:  # Only set the time on the first instance
                    self.all_nodes_started = True
                    self.all_nodes_started_time = self.get_clock().now()  # Record the time
            else:
                self.all_nodes_started = False  # Reset if a node stops
        else:
            self.m_service_started = False  # Reset if m_service stops
            self.all_nodes_started = False # Reset if m_service stops

    def flash_led(self):
        led_colors = []
        num_leds = 64  # Replace with your actual LED count
        brightness_factor = (math.sin(self.get_clock().now().nanoseconds / 1e9 * 4) + 1) / 2  # Pulse effect
        if not self.m_service_started:
            # Do nothing until m_service starts
            return
        brightness = int(255 * brightness_factor)  # Scale brightness 0-255
        if not self.all_nodes_started:
            # Flash red while waiting for other services
            for i in range(num_leds):
                if i % 2 == 0:
                    led_colors.append(LedColor(red=brightness, green=0, blue=0))
                else:
                    led_colors.append(LedColor(red=0, green=0, blue=0))
        else:
            # Solid green for 30 seconds
            # Calculate elapsed time since all nodes started
            elapsed_time = self.get_clock().now() - self.all_nodes_started_time
            if elapsed_time.nanoseconds / 1e9 < 10:  # Correct: Use elapsed_time.nanoseconds
                for i in range(num_leds):
                    led_colors.append(LedColor(red=0, green=brightness, blue=0))
            else:
                for i in range(num_leds):
                    led_colors.append(LedColor(red=0, green=0, blue=0))
                self.get_logger().info("All nodes started. Turning off LEDs.")
                self.destroy_timer(self.timer)
                cmsg = Chat()
                cmsg.chat_text = "Hey Shimmy, Say Hello and Tell me a little bit about yourself. Be brief"
                cmsg.sid_embedding = [0.0]
                cmsg.direction = 0
                self.chat_publisher_.publish(cmsg)
                msg = PaintLedColor()
                msg.colors = led_colors
                self.led_publisher_.publish(msg)
                return  # Stop the timer and exit

        msg = PaintLedColor()
        msg.colors = led_colors
        self.led_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    startup_led_controller = StartupLEDController()
    rclpy.spin(startup_led_controller)
    startup_led_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
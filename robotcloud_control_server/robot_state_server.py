import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from robotcloud_msgs.srv import RobotState

class RobotStateServer(Node):
    def __init__(self):
        super().__init__('robot_state_server')
        self.state = "OFF"

        # Latched-like behavior so UI gets last state immediately
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(String, '/robot_state_changed', qos)

        self.srv = self.create_service(RobotState, '/robot_state', self.handle_robot_state)
        self.srv = self.create_service(RobotState, '/robot_start', self.handle_robot_start)
        self.srv_stop = self.create_service(RobotState, '/robot_stop', self.handle_robot_stop)
        self.get_logger().info('robot_state_server up. Initial state: OFF')
        self._publish_state('Initial state')

    def _publish_state(self, reason=''):
        msg = String()
        msg.data = self.state
        self.pub.publish(msg)
        if reason:
            self.get_logger().info(f'State -> {self.state} ({reason})')

    def handle_robot_state(self, request, response):
        cmd = (request.command or '').upper().strip()

        if cmd == 'ON':
            if self.state != 'ON':
                # TODO: start your robot here (spawn process, call other services, etc.)
                self.state = 'ON'
                self._publish_state('turned ON')
                response.success = True
                response.status = self.state
                response.message = 'Robot started'
            else:
                response.success = True
                response.status = self.state
                response.message = 'Robot already ON'

        elif cmd == 'OFF':
            if self.state != 'OFF':
                # TODO: stop your robot gracefully here
                self.state = 'OFF'
                self._publish_state('turned OFF')
                response.success = True
                response.status = self.state
                response.message = 'Robot stopped'
            else:
                response.success = True
                response.status = self.state
                response.message = 'Robot already OFF'

        elif cmd == 'QUERY':
            response.success = True
            response.status = self.state
            response.message = f'Current state: {self.state}'

        else:
            response.success = False
            response.status = self.state
            response.message = 'Unknown command. Use ON, OFF, or QUERY.'

        return response

    def handle_robot_start(self, request, response):
        # Treat any call as "start"
        self.get_logger().info('robot_start service called')
        if self.state != 'ON':
            self.state = 'ON'
            self.get_logger().info('robot started')
            self._publish_state('turned ON (robot_start)')
            response.message = 'Robot started'
        else:
            self.get_logger().info('robot already ON')
            response.message = 'Robot already ON'
        response.success = True
        response.status = self.state
        return response

    def handle_robot_stop(self, request, response):
        # Treat any call as "stop"
        self.get_logger().info('robot_stop service called')
        if self.state != 'OFF':
            self.state = 'OFF'
            self.get_logger().info('the robot has stopped')
            self._publish_state('turned OFF (robot_stop)')
            response.message = 'Robot stopped'
        else:
            self.get_logger().info('the robot has stopped (already OFF)')
            response.message = 'Robot already OFF'
        response.success = True
        response.status = self.state
        return response

def main():
    rclpy.init()
    node = RobotStateServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

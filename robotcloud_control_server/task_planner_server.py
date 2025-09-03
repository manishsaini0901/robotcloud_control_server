import rclpy
from rclpy.node import Node
from robotcloud_msgs.srv import TaskPlanner


class TaskPlannerServer(Node):
    def __init__(self):
        super().__init__('task_planner_server')
        self.srv = self.create_service(TaskPlanner, "/task_planner", self.handle)
        self.get_logger().info("TaskPlanner service '/task_planner' ready")

    def handle(self, request: TaskPlanner.Request, response: TaskPlanner.Response):
        # Log the request for visibility during testing
        self.get_logger().info(
            "TaskPlanner request: command=%d, box_side=%d, object_id='%s', box_type=%d, max_parts=%d, start_layer=%d, start_obj=%d"
            % (
                int(request.command),
                int(request.box_side),
                str(request.object_id),
                int(request.box_type),
                int(request.maximum_parts_in_box),
                int(request.starting_layer_number),
                int(request.starting_object_number),
            )
        )

        # Minimal behavior: always accept and return success
        # You can extend this by branching on request.command to simulate different flows
        response.result = True
        return response


def main():
    rclpy.init()
    node = TaskPlannerServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

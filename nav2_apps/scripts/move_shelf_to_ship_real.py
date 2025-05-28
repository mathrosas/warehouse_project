import time
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from attach_service.srv import GoToLoading
from geometry_msgs.msg import Polygon, Point32, PoseStamped, Twist
from std_msgs.msg import String

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import Buffer, TransformListener


# Client for shelf lifting service
class ClientAsync(Node):
    def __init__(self):
        super().__init__('go_to_loading')
        self.client = self.create_client(GoToLoading, 'attach_service')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def send_request(self):
        req = GoToLoading.Request()
        self.future = self.client.call_async(req)

class PolygonPublisher(Node):
    def __init__(self):
        super().__init__('polygon_publisher')
        self.global_publisher_ = self.create_publisher(Polygon, '/global_costmap/footprint', 10)
        self.local_publisher_ = self.create_publisher(Polygon, '/local_costmap/footprint', 10)
        self.timer = self.create_timer(1, self.publish_polygon)

        self.robot_side = 0.250
        self.cart_side = 0.450

        self.robot_shape = [
            Point32(x=-self.robot_side, y=self.robot_side, z=0.0), 
            Point32(x=self.robot_side, y=self.robot_side, z=0.0),    
            Point32(x=self.robot_side, y=-self.robot_side, z=0.0),   
            Point32(x=-self.robot_side, y=-self.robot_side, z=0.0)   
        ]

        self.cart_shape = [
            Point32(x=-self.cart_side, y=self.cart_side, z=0.0), 
            Point32(x=self.cart_side, y=self.cart_side, z=0.0),    
            Point32(x=self.cart_side, y=-self.cart_side, z=0.0),   
            Point32(x=-self.cart_side, y=-self.cart_side, z=0.0)   
        ]

    def publish_polygon(self, mode):
        polygon_msg = Polygon()
        if (mode == 'robot'):
            polygon_msg.points = self.robot_shape
        elif (mode == 'cart'):
            polygon_msg.points = self.cart_shape
        else:
            self.get_logger().info(f'Invalid shape type {mode}')
            return None
        self.global_publisher_.publish(polygon_msg)
        self.local_publisher_.publish(polygon_msg)
        self.get_logger().info(f'Publishing polygon footprint of type {mode}')
        time.sleep(1)
        return None

class ElevatorPublisher(Node):
    def __init__(self):
        super().__init__('elevator_publisher')
        self.publisher_ = self.create_publisher(String, '/elevator_down', 10)
        self.lift_publisher_ = self.create_publisher(String, '/elevator_up', 10)

    def drop(self):
        msg = String()
        self.publisher_.publish(msg)
        self.publisher_.publish(msg)
        self.publisher_.publish(msg)
        self.publisher_.publish(msg)
        self.publisher_.publish(msg)
        time.sleep(6)
        self.get_logger().info(f'Shelf unloaded')
        return None

    def lift(self):
        msg = String()
        self.lift_publisher_.publish(msg)
        self.lift_publisher_.publish(msg)
        self.lift_publisher_.publish(msg)
        self.lift_publisher_.publish(msg)
        self.lift_publisher_.publish(msg)
        time.sleep(6)
        self.get_logger().info(f'Shelf loaded')
        return None

class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.duration = 10  # Set the duration for which the robot should move back

    def move_back(self):
        # Start time
        start_time = time.time()

        # Publish a message to move the robot backwards
        msg = Twist()
        msg.linear.x = -0.2  # Move backwards
        msg.angular.z = 0.05
        self.publisher_.publish(msg)
        self.get_logger().info('Moving the robot backwards')

        while time.time() - start_time < self.duration:
            self.publisher_.publish(msg)
            self.get_logger().info('Moving the robot backwards')
            time.sleep(0.1)  # Adjust the sleep time as needed for responsiveness

        # Stop the robot by publishing zero velocities
        self.publisher_.publish(Twist())
        self.get_logger().info('Stopping the robot')

# Shelf positions for picking
shelf_positions = {
    "init": [0.15, 0.0, 0.0, 1.0],
    "loading_position": [4.30, -1.00, -0.70, 0.70],
    "shipping_position": [2.20, 1.0, 0.70, 0.70]
    }

def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Set initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = shelf_positions['init'][0]
    initial_pose.pose.position.y = shelf_positions['init'][1]
    initial_pose.pose.orientation.z = shelf_positions['init'][2]
    initial_pose.pose.orientation.w = shelf_positions['init'][3]

    # Wait for navigation to activate fully
    navigator.waitUntilNav2Active()
    navigator.setInitialPose(initial_pose)

    # Got to the loading position
    request_item_location = 'loading_position'
    shelf_item_pose = PoseStamped()
    shelf_item_pose.header.frame_id = 'map'
    shelf_item_pose.header.stamp = navigator.get_clock().now().to_msg()
    shelf_item_pose.pose.position.x = shelf_positions[request_item_location][0]
    shelf_item_pose.pose.position.y = shelf_positions[request_item_location][1]
    shelf_item_pose.pose.orientation.z = shelf_positions[request_item_location][2]
    shelf_item_pose.pose.orientation.w = shelf_positions[request_item_location][3]
    print('Received request for item picking at ' + request_item_location + '.')

    for n in range(5):
        navigator.goToPose(shelf_item_pose)
        i = 0
        while not navigator.isTaskComplete():
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival at ' + request_item_location +
                    ' for worker: ' + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f'Task completed successfully on {n} attempt.')
            break

        elif result == TaskResult.CANCELED:
            print('Task at ' + request_item_location +
                ' was canceled. Returning to staging point...')
            initial_pose.header.stamp = navigator.get_clock().now().to_msg()
            navigator.goToPose(initial_pose)
            break

        elif result == TaskResult.FAILED:
            print('Task at ' + request_item_location + f' failed on its {n + 1} attempt.')
            if n + 1 == 5: exit(-1)
            time.sleep(1.0)

    while not navigator.isTaskComplete():
        pass

    # Instance the elevator publisher
    elevator_publisher = ElevatorPublisher()

    # Instance the footprint publisher
    footprint_publisher = PolygonPublisher()
    
    # Instance the service client for shelf lifting
    for n in range(5):
        client = ClientAsync()
        print(f'Attempt {n+1}: Calling shelf lifting service.')
        client.send_request()

        while rclpy.ok():
            rclpy.spin_once(client)
            if client.future.done():
                try:
                    response = client.future.result()
                except Exception as e:
                    client.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    client.get_logger().info(f'Result of service call: {response.complete}')
                    
                break

        client.destroy_node()
        if response.complete == True:
            elevator_publisher.lift()
            break

    if n + 1 == 5: exit(-1)

    # Handle lift and backwards movement
    footprint_publisher.publish_polygon('cart')
    mover = RobotMover()
    mover.move_back()

    # Got to the shipping position
    request_item_location = 'shipping_position'
    shelf_item_pose = PoseStamped()
    shelf_item_pose.header.frame_id = 'map'
    shelf_item_pose.header.stamp = navigator.get_clock().now().to_msg()
    shelf_item_pose.pose.position.x = shelf_positions[request_item_location][0]
    shelf_item_pose.pose.position.y = shelf_positions[request_item_location][1]
    shelf_item_pose.pose.orientation.z = shelf_positions[request_item_location][2]
    shelf_item_pose.pose.orientation.w = shelf_positions[request_item_location][3]
    print('Moving robot to ' + request_item_location + '.')
    navigator.waitUntilNav2Active()
    navigator.goToPose(shelf_item_pose)

    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival at ' + request_item_location +
                  ' for worker: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Unloading the shelf.')
        footprint_publisher.publish_polygon('robot')
        elevator_publisher.drop()
        mover.move_back()

    elif result == TaskResult.CANCELED:
        print('Task at ' + request_item_location +
              ' was canceled. Returning to staging point...')
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(initial_pose)

    elif result == TaskResult.FAILED:
        print('Task at ' + request_item_location + ' failed!')
        exit(-1)

     # Got to the init position
    request_item_location = 'init'
    shelf_item_pose = PoseStamped()
    shelf_item_pose.header.frame_id = 'map'
    shelf_item_pose.header.stamp = navigator.get_clock().now().to_msg()
    shelf_item_pose.pose.position.x = shelf_positions[request_item_location][0]
    shelf_item_pose.pose.position.y = shelf_positions[request_item_location][1]
    shelf_item_pose.pose.orientation.z = shelf_positions[request_item_location][2]
    shelf_item_pose.pose.orientation.w = shelf_positions[request_item_location][3]
    print('Moving robot to ' + request_item_location + '.')
    navigator.goToPose(shelf_item_pose)

    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival at ' + request_item_location +
                  ' for worker: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Task finished.')

    elif result == TaskResult.CANCELED:
        print('Task at ' + request_item_location +
              ' was canceled. Returning to staging point...')
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(initial_pose)

    elif result == TaskResult.FAILED:
        print('Task at ' + request_item_location + ' failed!')
        exit(-1)
    
    exit(0)


if __name__ == '__main__':
    main()

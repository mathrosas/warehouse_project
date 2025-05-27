#include "attach_service/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <vector>

using namespace std::placeholders;

class AttachService : public rclcpp::Node {
public:
  AttachService() : Node("attach_service_server") {
    // TF buffer & listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_static_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Service
    srv_ = this->create_service<attach_service::srv::GoToLoading>(
        "/attach_service",
        std::bind(&AttachService::handle_service, this, _1, _2));

    // Laser subscription
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&AttachService::laser_callback, this, _1));

    // Publishers
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/diffbot_base_controller/cmd_vel_unstamped", 1);
    elevator_pub_ =
        this->create_publisher<std_msgs::msg::String>("/elevator_up", 1);

    RCLCPP_INFO(this->get_logger(), "Attach service (sync) is ready.");
  }

private:
  // Laser callback: detect two leg indices
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    last_ranges_ = msg->ranges;
    last_intensities_ = msg->intensities;
    leg_indices_.clear();
    bool in_blob = false;
    int blob_start = 0;
    const double INT_THRESH = 100.0;
    for (int i = 0; i < (int)last_intensities_.size(); ++i) {
      if (last_intensities_[i] > INT_THRESH) {
        if (!in_blob) {
          in_blob = true;
          blob_start = i;
        }
      } else if (in_blob) {
        in_blob = false;
        leg_indices_.push_back((blob_start + i) / 2);
      }
    }
    RCLCPP_INFO(get_logger(), "Scan: ranges=%zu intensities=%zu",
                msg->ranges.size(), msg->intensities.size());
  }

  // Synchronous handle_service: block until attach complete
  void handle_service(
      const std::shared_ptr<attach_service::srv::GoToLoading::Request> /*req*/,
      std::shared_ptr<attach_service::srv::GoToLoading::Response> response) {
    // 1) Check for two legs
    if (leg_indices_.size() != 2) {
      RCLCPP_WARN(this->get_logger(),
                  "Need exactly two reflective blobs, found %zu.",
                  leg_indices_.size());
      response->complete = false;
      return;
    }

    // 2) Publish cart_frame once
    if (!publish_cart_frame()) {
      response->complete = false;
      return;
    }

    // 3) Approach the cart_frame synchronously
    rclcpp::Rate rate(10); // 10 Hz loop
    while (rclcpp::ok()) {
      geometry_msgs::msg::TransformStamped t;
      try {
        t = tf_buffer_->lookupTransform("robot_base_footprint", "cart_frame",
                                        tf2::TimePointZero);
      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Waiting for cart_frame TF: %s",
                    ex.what());
        rate.sleep();
        continue;
      }

      // Compute offset (0.5 m for gripper)
      double x = t.transform.translation.x + 0.3;
      double y = t.transform.translation.y;
      double dist = std::hypot(x, y);
      double yaw_err = std::atan2(y, x);

      if (dist <= 0.05) {
        // Stop
        vel_pub_->publish(geometry_msgs::msg::Twist());
        break;
      }

      // Publish command
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = 0.2;
      cmd.angular.z = -0.2 * yaw_err;
      vel_pub_->publish(cmd);
      rate.sleep();
    }

    // 4) Attach complete: publish elevator_up and respond
    RCLCPP_INFO(this->get_logger(), "Attach complete");
    std_msgs::msg::String up_msg;
    up_msg.data = "up";
    elevator_pub_->publish(up_msg);

    response->complete = true;
  }

  // Broadcast cart_frame based on last laser readings
  bool publish_cart_frame() {
    if (leg_indices_.size() < 2)
      return false;
    int idx1 = leg_indices_[0], idx2 = leg_indices_[1];
    double angle_min = -M_PI;
    double angle_inc = (2 * M_PI) / last_ranges_.size();
    double a1 = angle_min + idx1 * angle_inc;
    double a2 = angle_min + idx2 * angle_inc;
    double r1 = last_ranges_[idx1];
    double r2 = last_ranges_[idx2];
    double mid_x = (r1 * cos(a1) + r2 * cos(a2)) / 2.0;
    double mid_y = (r1 * sin(a1) + r2 * sin(a2)) / 2.0;

    geometry_msgs::msg::PointStamped laser_pt, odom_pt;
    laser_pt.header.frame_id = "robot_front_laser_base_link";
    laser_pt.header.stamp = this->now();
    laser_pt.point.x = mid_x;
    laser_pt.point.y = mid_y;

    try {
      auto tf = tf_buffer_->lookupTransform(
          "odom", "robot_front_laser_base_link", tf2::TimePointZero,
          tf2::durationFromSec(0.5));
      tf2::doTransform(laser_pt, odom_pt, tf);
      geometry_msgs::msg::TransformStamped cart_tf;
      cart_tf.header.stamp = this->now();
      cart_tf.header.frame_id = "odom";
      cart_tf.child_frame_id = "cart_frame";
      cart_tf.transform.translation.x = odom_pt.point.x;
      cart_tf.transform.translation.y = odom_pt.point.y;
      cart_tf.transform.rotation.w = 1.0;
      tf_static_broadcaster_->sendTransform(cart_tf);
      return true;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "TF error: %s", ex.what());
      return false;
    }
  }

  // Members
  rclcpp::Service<attach_service::srv::GoToLoading>::SharedPtr srv_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elevator_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  std::vector<float> last_ranges_;
  std::vector<float> last_intensities_;
  std::vector<int> leg_indices_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AttachService>());
  rclcpp::shutdown();
  return 0;
}

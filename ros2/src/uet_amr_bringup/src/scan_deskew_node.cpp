#include <memory>
#include <string>

#include "laser_geometry/laser_geometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// Re-projects each point of an incoming LaserScan using its own capture
// timestamp (header.stamp + index * time_increment) into a non-rotating
// fixed frame, instead of the single shared pose slam_toolbox/RViz apply to
// the whole message. Removes the per-scan motion-distortion smear that
// shows up as a sunburst artifact in the map when the robot rotates during
// the ~100-200ms it takes the lidar to complete one sweep.
class ScanDeskewNode : public rclcpp::Node
{
public:
  ScanDeskewNode()
  : Node("scan_deskew_node")
  {
    fixed_frame_ = this->declare_parameter<std::string>("fixed_frame", "odom");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "scan_deskewed_points", rclcpp::SensorDataQoS());

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      std::bind(&ScanDeskewNode::scan_callback, this, std::placeholders::_1));
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    try {
      projector_.transformLaserScanToPointCloud(fixed_frame_, *scan, cloud, *tf_buffer_);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Could not deskew scan into '%s': %s", fixed_frame_.c_str(), ex.what());
      return;
    }
    cloud_pub_->publish(cloud);
  }

  std::string fixed_frame_;
  laser_geometry::LaserProjection projector_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanDeskewNode>());
  rclcpp::shutdown();
  return 0;
}

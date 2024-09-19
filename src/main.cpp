#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_ros/transforms.hpp>
   
class VolumeCalcNode : public rclcpp::Node
{
public:
  VolumeCalcNode() : Node("VolumeCalculatorNode")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "input_point_cloud", 10, std::bind(&VolumeCalcNode::topic_callback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output_point_cloud", 10);
  }

private:
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setInputCloud(cloud);
    chull.reconstruct(*cloud_hull);

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud_hull, output);
    output.header = msg->header;

    publisher_->publish(output);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VolumeCalcNode>());
  rclcpp::shutdown();
  return 0;
}

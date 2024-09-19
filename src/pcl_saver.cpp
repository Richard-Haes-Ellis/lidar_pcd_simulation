#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <std_srvs/srv/empty.hpp>  // Standard service for triggering the save

class PointCloudSaverService : public rclcpp::Node
{
public:
    PointCloudSaverService() : Node("pointcloud_saver_service"), latest_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
    {
        // Subscribe to the pointcloud topic
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidar/points", 10, std::bind(&PointCloudSaverService::pointcloud_callback, this, std::placeholders::_1));

        // Create the service that triggers saving the pointcloud
        save_service_ = this->create_service<std_srvs::srv::Empty>(
            "save_pointcloud", std::bind(&PointCloudSaverService::save_pointcloud_callback, this, std::placeholders::_1, std::placeholders::_2));
        
        RCLCPP_INFO(this->get_logger(), "PointCloudSaverService node is up and running.");
    }

private:
    // Callback when a new point cloud message is received
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert PointCloud2 message to PCL format
        pcl::fromROSMsg(*msg, *latest_cloud_);
    }

    // Callback when the save_pointcloud service is called
    void save_pointcloud_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                  std::shared_ptr<std_srvs::srv::Empty::Response> response)
    {
        // Save the latest point cloud data to a file
        std::string filename = "saved_pointcloud.pcd";
        if (pcl::io::savePCDFileASCII(filename, *latest_cloud_) == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Point cloud saved successfully to %s", filename.c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to save point cloud to %s", filename.c_str());
        }
    }

    // The latest received point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr latest_cloud_;

    // Subscription to the PointCloud2 topic
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

    // Service to save the point cloud
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudSaverService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

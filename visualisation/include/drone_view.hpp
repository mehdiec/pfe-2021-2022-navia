#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <fstream>

#include "gnuplot-iostream.h"

#include "rclcpp/rclcpp.hpp"
#include "rcpputils/endian.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "std_msgs/msg/empty.hpp"
#include <sensor_msgs/msg/point_cloud.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

//=============================================================================
// DroneView node
//=============================================================================
class DroneViz : public rclcpp::Node
{
    sensor_msgs::msg::CameraInfo caminfo;
    sensor_msgs::msg::Image full_image;

    int im_id = 0;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    

    // Publications
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pcl_pub;

    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_input;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscription_caminfo;
    rclcpp::TimerBase::SharedPtr timer_pcl;

public:
    explicit DroneViz();

private:
    // Callbacks
    void on_image(sensor_msgs::msg::Image::SharedPtr msg);
    void on_caminfo(sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void ground_points(); // std_msgs::msg::Float32 &height, sensor_msgs::msg::CompressedImage &full_image);
    geometry_msgs::msg::TransformStamped get_transformation(std::string source_frame , std::string target_frame);
};

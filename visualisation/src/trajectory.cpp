#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud.hpp>
#include "rcpputils/endian.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;


class TrajectoryViz : public rclcpp::Node
{

public : 
    TrajectoryViz() : Node("TrajectoryViz")
    {

        pcl_pub = create_publisher<sensor_msgs::msg::PointCloud>("pcl_trajectory", 100);

        pcl_sub = create_subscription<sensor_msgs::msg::PointCloud>("pcl_out", 100, std::bind(&TrajectoryViz::on_pcl, this, _1));


    }

private:

    // variables
    std::vector< std::vector<geometry_msgs::msg::Point32> > cloud_;
    int nb_msg = 0 ; 
    // Subscribtions
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr pcl_sub;

    //Publications
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pcl_pub;

    //Useful function 

    std::vector<geometry_msgs::msg::Point32> flatten(std::vector<std::vector<geometry_msgs::msg::Point32>>& v) {
    std::size_t total_size = 0;
    for (const auto& sub : v)
        total_size += sub.size(); 
    std::vector<geometry_msgs::msg::Point32> result;
    result.reserve(total_size);
    for (const auto& sub : v)
        result.insert(result.end(), sub.begin(), sub.end());
    return result;
}

    // Callbacks
    void on_pcl(sensor_msgs::msg::PointCloud::SharedPtr msg)
    {
        nb_msg+=1;
        std::vector<geometry_msgs::msg::Point32> current_cloud;
        current_cloud = msg->points;
        if(nb_msg<=20)
            cloud_.push_back(current_cloud);

        
        if(nb_msg>20)
        {
            cloud_.erase(cloud_.begin());

            std::vector<geometry_msgs::msg::Point32> cloud_points = TrajectoryViz::flatten(cloud_);


            sensor_msgs::msg::PointCloud::SharedPtr pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud>();
            pc2_msg_->header.frame_id = "base";
            pc2_msg_->header.stamp = now();
            pc2_msg_->points = cloud_points;
            pcl_pub->publish(*pc2_msg_);
            RCLCPP_INFO(this->get_logger(), "[Trajectory] Published Point Cloud.");

            cloud_.push_back(current_cloud);
        }
    }
    

};



int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryViz>());
  rclcpp::shutdown();

  return 0;
}
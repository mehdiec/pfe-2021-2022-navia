
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "drone_view.hpp"
#include <opencv2/opencv.hpp>
using namespace std::placeholders;
using namespace std::chrono_literals;
/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

//=============================================================================
// DroneBase node
//=============================================================================

DroneViz::DroneViz() : Node("drone_view")
{

  pcl_pub = create_publisher<sensor_msgs::msg::PointCloud>("pcl_out", 100);

  subscription_input = create_subscription<sensor_msgs::msg::Image>("image_in", 100, std::bind(&DroneViz::on_image, this, _1));
  subscription_caminfo = create_subscription<sensor_msgs::msg::CameraInfo>("/sensor_msgs/CameraInfo", 100, std::bind(&DroneViz::on_caminfo, this, _1));

  //timer_pcl = create_wall_timer(500ms, std::bind(&DroneViz::ground_points, this));
  // tf 
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


}

geometry_msgs::msg::TransformStamped DroneViz::get_transformation(std::string source_frame , std::string target_frame)
{

 geometry_msgs::msg::TransformStamped transformStamped;
    

        //while(!tf_buffer_->canTransform(target_frame, source_frame,tf2::TimePointZero))
        while(true)
        {
          //std::this_thread::sleep_for(std::chrono::milliseconds(2));
        //transformStamped = tf_buffer_->lookupTransform(target_frame, source_frame,tf2::TimePointZero);
        try{
          transformStamped = tf_buffer_->lookupTransform(target_frame, source_frame,tf2::TimePointZero);
            return transformStamped;
        } catch (...) {
          RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s",target_frame.c_str(), source_frame.c_str());
        }

        }
}
void DroneViz::on_image(sensor_msgs::msg::Image::SharedPtr msg)
{
  full_image = (*msg);
  //RCLCPP_INFO(this->get_logger(), "Received camera image");
  this->ground_points();
  return;
}

void DroneViz::on_caminfo(sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  caminfo = *msg  ;//->data;
  //RCLCPP_INFO(this->get_logger(), "Received camera infos");
  return;
}

void DroneViz::ground_points() // std_msgs::msg::Float32 &height, sensor_msgs::msg::Image &full_image)
{

  
  

  auto Camera  = image_geometry::PinholeCameraModel ();
  

   if (Camera.fromCameraInfo(caminfo)==true) 
   {


    auto transformStamped = DroneViz::get_transformation(std::string("anafi") , std::string("base")); 

     cv_bridge::CvImagePtr image = cv_bridge::toCvCopy(full_image, "mono8");
    
    
      cv::Mat im = image->image;
      
    
    // save image 
    //std::string im_path = "/home/rudolf/Documents/images_vid/seg/image" + std::to_string(im_id) + ".jpg";
    
    //cv::imwrite(im_path, im);
      
      
      
      int h = im.rows;
      int w = im.cols;

    std::vector<geometry_msgs::msg::Point32> cloud_;


    double pixel_size_x = 2.6e-7;
    double pixel_size_y = 2.6e-7;
    double f_x = Camera.fx()*pixel_size_x;
    double f_y = Camera.fy()*pixel_size_y;
    double e_y = h*0.5*pixel_size_y;
    double e_x = w*0.5*pixel_size_x;


    cv::Mat plot_images = cv::Mat::ones(cv::Size(w,h),CV_8UC1);
    

    uchar* p;
    for( int i = 0; i < h; ++i)
    {
        p = im.ptr<uchar>(i);
        for ( int j = 0; j < w; ++j)
        {
            if (p[j]==255){

          auto t = transformStamped.transform.translation ;
          geometry_msgs::msg::Point pt;
          pt.x = -t.z;
          double depth = (t.z/(f_y*i*pixel_size_y) )*(f_y*f_y-e_y*i*pixel_size_y);
          pt.z= depth;
          pt.y = (j*pixel_size_x-e_x)*depth/f_x;


          geometry_msgs::msg::PointStamped pt_anafi,pt_base;

          pt_anafi.header.frame_id = "anafi";
          pt_anafi.header.stamp = now();
          pt_anafi.point = pt;


          tf2::doTransform(pt_anafi,pt_base,transformStamped );
          
          geometry_msgs::msg::Point32 pub_point;
          pub_point.x = pt_base.point.x;
          pub_point.y = pt_base.point.y;
          pub_point.z = pt_base.point.z;
          cloud_.push_back(pub_point);

          
          plot_images.ptr<uchar>( (int)(h-1-(pt.z-1)*h/1.5) )[ (int)((pt.y+0.5)*w/1.2) ] = 255;
          
        }

      }
    }


    //std::string im_path2 = "/home/rudolf/Documents/images_vid/coord/image" + std::to_string(im_id) + ".jpg";
    
    //cv::imwrite(im_path2, plot_images);

    im_id+=1;
    
        
    
    sensor_msgs::msg::PointCloud::SharedPtr pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud>();
    pc2_msg_->header.frame_id = "base";
    pc2_msg_->header.stamp = now();
    pc2_msg_->points = std::move(cloud_);
    pcl_pub->publish(*pc2_msg_);
    RCLCPP_INFO(this->get_logger(), "[RRT*] Published Point Cloud.");
   }
   else{
     RCLCPP_INFO(this->get_logger(), "Cannot set cameraInfo");
   }


}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneViz>());
  rclcpp::shutdown();

  return 0;
}

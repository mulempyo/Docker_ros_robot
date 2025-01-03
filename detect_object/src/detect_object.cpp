#include <detect_object/detect_object.h>
#include "ros/ros.h"
#include <costmap_2d/costmap_2d_ros.h>
#include <std_msgs/Float64.h>
#include <gb_visual_detection_3d_msgs/BoundingBoxes3d.h>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <memory>

namespace detect{

Detect::Detect(){
  tf_.reset(new tf2_ros::Buffer);
  tf_->setUsingDedicatedThread(true);
  tfl_.reset(new tf2_ros::TransformListener(*tf_));
}

Detect::~Detect(){
   
}

void Detect::boundingBoxCallback(const gb_visual_detection_3d_msgs::BoundingBoxes3d::ConstPtr& msg)
  {
    for (const auto& box : msg->bounding_boxes)
    {
      if(box.Class == "person" && box.probability > 50){
           person.data = 1.0;
         }else{
           person.data = 0.0; 
         }

         pub1.publish(person);
         }
  }
} //namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bounding_boxes_3d");
    ros::NodeHandle nh;

    detect::Detect detector;
    
    detector.sub = nh.subscribe("/darknet_ros_3d/bounding_boxes", 10, &detect::Detect::boundingBoxCallback, &detector);
    detector.pub1 = nh.advertise<std_msgs::Float64>("person_probabilty",10);

    ros::spin();
    return 0;
}

#include <detect_object/detect_object.h>
#include <vector>
#include <map>
#include <Eigen/Core>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <std_msgs/Int64.h>
#include <gb_visual_detection_3d_msgs/BoundingBoxes3d.h>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <memory>
#include <cuda_runtime.h>

#define MAX_BOXES 100 

extern void runBoundingBoxKernel(float *d_obj_x, float *d_obj_y, float *d_obj_z,
                                     float *d_probabilities, int *d_class_ids,
                                     float *d_distances, int *d_person_detected, int num_boxes);

namespace detect {

Detect::Detect(ros::NodeHandle& nh) {
    tf_.reset(new tf2_ros::Buffer);
    tf_->setUsingDedicatedThread(true);
    tfl_.reset(new tf2_ros::TransformListener(*tf_));
    ROS_WARN("in detect");
    sub = nh.subscribe("/darknet_ros_3d/bounding_boxes", 10, &Detect::boundingBoxCallback, this);
    pub1 = nh.advertise<std_msgs::Int64>("person_probability", 10);
}

Detect::~Detect() {}

void Detect::boundingBoxCallback(const gb_visual_detection_3d_msgs::BoundingBoxes3d::ConstPtr& msg) {
    ROS_WARN("in callback");
    int num_boxes = msg->bounding_boxes.size();

    num_boxes = std::min(num_boxes, MAX_BOXES);  

    float h_obj_x[MAX_BOXES], h_obj_y[MAX_BOXES], h_obj_z[MAX_BOXES];
    float h_distances[MAX_BOXES];
    int h_person_detected[MAX_BOXES];
    int h_class_ids[MAX_BOXES];
    float h_probabilities[MAX_BOXES];

    float *d_obj_x, *d_obj_y, *d_obj_z, *d_distances, *d_probabilities;
    int *d_person_detected, *d_class_ids;

    cudaMalloc((void**)&d_obj_x, num_boxes * sizeof(float));
    cudaMalloc((void**)&d_obj_y, num_boxes * sizeof(float));
    cudaMalloc((void**)&d_obj_z, num_boxes * sizeof(float));
    cudaMalloc((void**)&d_distances, num_boxes * sizeof(float));
    cudaMalloc((void**)&d_person_detected, num_boxes * sizeof(int));
    cudaMalloc((void**)&d_class_ids, num_boxes * sizeof(int));
    cudaMalloc((void**)&d_probabilities, num_boxes * sizeof(float));

    for (int i = 0; i < num_boxes; i++) {
        const auto& box = msg->bounding_boxes[i];
        h_obj_x[i] = (box.xmax + box.xmin) / 2.0;
        h_obj_y[i] = (box.ymax + box.ymin) / 2.0;
        h_obj_z[i] = (box.zmax + box.zmin) / 2.0;
        h_class_ids[i] = (box.Class == "person") ? 0 : -1;
        h_probabilities[i] = box.probability;
    }

    cudaMemcpy(d_obj_x, h_obj_x, num_boxes * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_obj_y, h_obj_y, num_boxes * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_obj_z, h_obj_z, num_boxes * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_class_ids, h_class_ids, num_boxes * sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(d_probabilities, h_probabilities, num_boxes * sizeof(float), cudaMemcpyHostToDevice);

    runBoundingBoxKernel(d_obj_x, d_obj_y, d_obj_z,
                         d_probabilities, d_class_ids,
                         d_distances, d_person_detected,
                         num_boxes);

    cudaMemcpy(h_distances, d_distances, num_boxes * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(h_person_detected, d_person_detected, num_boxes * sizeof(int), cudaMemcpyDeviceToHost);

    cudaFree(d_obj_x);
    cudaFree(d_obj_y);
    cudaFree(d_obj_z);
    cudaFree(d_distances);
    cudaFree(d_person_detected);
    cudaFree(d_class_ids);
    cudaFree(d_probabilities);

    std_msgs::Int64 person;
    person.data = 0;
    ROS_WARN("person 0");
    for (int i = 0; i < num_boxes; i++) {
        if (h_person_detected[i] == 1) {
            person.data = 1;
            ROS_WARN("person 1");
            break;
        }
    }

    pub1.publish(person);
}

} // namespace detect

int main(int argc, char** argv) {
    ros::init(argc, argv, "bounding_boxes_3d");
    ros::NodeHandle nh;

    detect::Detect detector(nh);

    ros::spin();
    return 0;
}


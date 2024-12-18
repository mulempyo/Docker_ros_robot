// visual_odometry_node.cpp

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>

class VisualOdometryNode {
public:
    VisualOdometryNode(ros::NodeHandle& nh) : it_(nh), first_frame_(true), x_(0.0), y_(0.0), yaw_(0.0) {
       
        image_sub_ = it_.subscribe("/camera/image_raw", 1, &VisualOdometryNode::imageCallback, this);

     
        odom_pub_ = nh.advertise<nav_msgs::Odometry>("visual_odometry", 10);

       
        orb_ = cv::ORB::create(2000);
        bf_matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING, true);
    }

private:
    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat current_image = cv_ptr->image;
        std::vector<cv::KeyPoint> current_keypoints;
        cv::Mat current_descriptors;

        orb_->detectAndCompute(current_image, cv::Mat(), current_keypoints, current_descriptors);

        if (first_frame_) {
            previous_image_ = current_image.clone();
            previous_keypoints_ = current_keypoints;
            previous_descriptors_ = current_descriptors.clone();
            first_frame_ = false;
            return;
        }

        std::vector<cv::DMatch> matches;
        bf_matcher_->match(previous_descriptors_, current_descriptors_, matches);

        std::sort(matches.begin(), matches.end(), [](const cv::DMatch& a, const cv::DMatch& b) {
            return a.distance < b.distance;
        });

        size_t num_good_matches = matches.size() * 0.5;
        matches.erase(matches.begin() + num_good_matches, matches.end());

        std::vector<cv::Point2f> prev_pts;
        std::vector<cv::Point2f> curr_pts;

        for (const auto& match : matches) {
            prev_pts.push_back(previous_keypoints_[match.queryIdx].pt);
            curr_pts.push_back(current_keypoints[match.trainIdx].pt);
        }

        if (prev_pts.size() >= 5) {
            cv::Mat mask;
            cv::Mat E = cv::findEssentialMat(curr_pts, prev_pts, 700, cv::Point2d(current_image.cols / 2, current_image.rows / 2), cv::RANSAC, 0.999, 1.0, mask);

            cv::Mat R, t;
            int inliers = cv::recoverPose(E, curr_pts, prev_pts, R, t, 700, cv::Point2d(current_image.cols / 2, current_image.rows / 2), mask);

            double delta_x = t.at<double>(0);
            double delta_y = t.at<double>(1);
            double delta_yaw = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0));

            x_ += delta_x;
            y_ += delta_y;
            yaw_ += delta_yaw;

            nav_msgs::Odometry odom_msg;
            odom_msg.header.stamp = msg->header.stamp;
            odom_msg.header.frame_id = "odom";
            odom_msg.child_frame_id = "base_link";

            odom_msg.pose.pose.position.x = x_;
            odom_msg.pose.pose.position.y = y_;
            odom_msg.pose.pose.position.z = 0.0;

            tf2::Quaternion quat;
            quat.setRPY(0, 0, yaw_);
            quat.normalize();
            odom_msg.pose.pose.orientation.x = quat.x();
            odom_msg.pose.pose.orientation.y = quat.y();
            odom_msg.pose.pose.orientation.z = quat.z();
            odom_msg.pose.pose.orientation.w = quat.w();

            for (int i = 0; i < 36; ++i) {
                odom_msg.pose.covariance[i] = 0.0;
            }
            odom_msg.pose.covariance[0] = 0.1;  // x
            odom_msg.pose.covariance[7] = 0.1;  // y
            odom_msg.pose.covariance[35] = 0.1; // yaw

            odom_msg.twist.twist.linear.x = delta_x / 0.02; // assuming 50 Hz
            odom_msg.twist.twist.linear.y = delta_y / 0.02;
            odom_msg.twist.twist.angular.z = delta_yaw / 0.02;

            odom_pub_.publish(odom_msg);
        }

        previous_image_ = current_image.clone();
        previous_keypoints_ = current_keypoints;
        previous_descriptors_ = current_descriptors.clone();
    }

    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher odom_pub_;

    cv::Ptr<cv::ORB> orb_;
    cv::Ptr<cv::BFMatcher> bf_matcher_;

    cv::Mat previous_image_;
    std::vector<cv::KeyPoint> previous_keypoints_;
    cv::Mat previous_descriptors_;
    cv::Mat current_descriptors_;

    bool first_frame_;
    double x_, y_, yaw_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "visual_odometry_node");
    ros::NodeHandle nh;

    VisualOdometryNode vo_node(nh);

    ros::spin();

    return 0;
}


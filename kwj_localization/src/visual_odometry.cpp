#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf2/LinearMath/Quaternion.h>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <tf2/LinearMath/Matrix3x3.h>

class VisualOdometryWithIMU {
public:
    VisualOdometryWithIMU(ros::NodeHandle& nh) : it_(nh), first_frame_(true), x_(0.0), y_(0.0), yaw_(0.0),
                                                    roll_(0.0), pitch_(0.0) {
        image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &VisualOdometryWithIMU::imageCallback, this);
        depth_sub_ = it_.subscribe("/camera/depth/image_rect_raw", 1, &VisualOdometryWithIMU::depthCallback, this);
        imu_sub_ = nh.subscribe("/imu/data", 10, &VisualOdometryWithIMU::imuCallback, this);
        odom_pub_ = nh.advertise<nav_msgs::Odometry>("visual_odometry", 10);

        orb_ = cv::ORB::create(2000);
        matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING, true);
    }

private:
    cv::Mat latest_depth_;

    void depthCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            latest_depth_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("Depth cv_bridge exception: %s", e.what());
        }
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);
        tf2::Matrix3x3(q).getRPY(roll_, pitch_, dummy_yaw_);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {

        if (latest_depth_.empty()) {
            ROS_WARN("No depth image yet, skipping frame");
            return;
        }
        
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat img = cv_ptr->image;

        int depth_rows = latest_depth_.rows;
        int depth_cols = latest_depth_.cols;

        if (img.rows != depth_rows || img.cols != depth_cols) {
            ROS_WARN("Image and depth dimensions mismatch (%dx%d vs %dx%d), skipping frame",
            img.cols, img.rows, depth_cols, depth_rows);
            return;
        }

        cv::Mat mask(img.rows, img.cols, CV_8UC1, cv::Scalar(0));
        for (int y = 0; y < img.rows; ++y) {
            for (int x = 0; x < img.cols; ++x) {
                uint16_t d = latest_depth_.at<uint16_t>(y, x); 
                if (d > 500 && d < 3000) {
                    mask.at<uchar>(y, x) = 255; 
                }
            }
        }

        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        orb_->detectAndCompute(img, mask, keypoints, descriptors);

        if (first_frame_) {
            previous_image_ = img.clone();
            previous_keypoints_ = keypoints;
            previous_descriptors_ = descriptors.clone();
            first_frame_ = false;
            return;
        }

        if (first_frame_) {
            previous_image_ = img.clone();
            previous_keypoints_ = keypoints;
            previous_descriptors_ = descriptors.clone();
            first_frame_ = false;
            return;
        }

        std::vector<cv::DMatch> matches;
        matcher_->match(previous_descriptors_, descriptors, matches);

        std::sort(matches.begin(), matches.end(), [](const cv::DMatch& a, const cv::DMatch& b) {
            return a.distance < b.distance;
        });

        matches.erase(matches.begin() + matches.size() / 2, matches.end());

        std::vector<cv::Point2f> pts_prev, pts_curr;
        for (const auto& match : matches) {
            pts_prev.push_back(previous_keypoints_[match.queryIdx].pt);
            pts_curr.push_back(keypoints[match.trainIdx].pt);
        }

        if (pts_prev.size() >= 5) {
            cv::Mat mask_pose;
            cv::Mat E = cv::findEssentialMat(pts_curr, pts_prev, 700,
                                             cv::Point2d(img.cols / 2, img.rows / 2),
                                             cv::RANSAC, 0.999, 1.0, mask_pose);

            cv::Mat R, t;
            int inliers = cv::recoverPose(E, pts_curr, pts_prev, R, t, 700,
                                          cv::Point2d(img.cols / 2, img.rows / 2), mask_pose);

            if (inliers < 10) {
                ROS_WARN("Too few inliers (%d), skipping this frame", inliers);
                return;
            }

            double dx = t.at<double>(0);
            double dy = t.at<double>(1);
            double dyaw = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0));

            x_ += dx;
            y_ += dy;
            yaw_ += dyaw;

            nav_msgs::Odometry odom;
            odom.header.stamp = msg->header.stamp;
            odom.header.frame_id = "odom";
            odom.child_frame_id = "base_footprint";
            odom.pose.pose.position.x = x_;
            odom.pose.pose.position.y = y_;

            tf2::Quaternion q;
            q.setRPY(roll_, pitch_, yaw_);
            q.normalize();
            odom.pose.pose.orientation.x = q.x();
            odom.pose.pose.orientation.y = q.y();
            odom.pose.pose.orientation.z = q.z();
            odom.pose.pose.orientation.w = q.w();

            odom_pub_.publish(odom);
        }

        previous_image_ = img.clone();
        previous_keypoints_ = keypoints;
        previous_descriptors_ = descriptors.clone();
    }

    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Subscriber depth_sub_;
    ros::Subscriber imu_sub_;
    ros::Publisher odom_pub_;

    cv::Ptr<cv::ORB> orb_;
    cv::Ptr<cv::BFMatcher> matcher_;

    cv::Mat previous_image_;
    std::vector<cv::KeyPoint> previous_keypoints_;
    cv::Mat previous_descriptors_;

    bool first_frame_;
    double x_, y_, yaw_;
    double roll_, pitch_, dummy_yaw_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "visual_odometry_node");
    ros::NodeHandle nh;
    VisualOdometryWithIMU node(nh);
    ros::spin();
    return 0;
}


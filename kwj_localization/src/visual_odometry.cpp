/*
  참고: 동적 환경에서 강인한 영상특징을 이용한 스테레오 비전 기반의 비주얼 오도메트리 = 
  Stereo vision-based visual odometry using robust visual features in dynamic environment
*/
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
#include <angles/angles.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

class VisualOdometryWithIMU {
public:
    VisualOdometryWithIMU(ros::NodeHandle& nh) : it_(nh), first_frame_(true), x_(0.0), y_(0.0), yaw_(0.0),
                                                    roll_(0.0), pitch_(0.0) {
        rgb_sub_.subscribe(nh, "/camera/color/image_raw", 1);
        depth_sub_.subscribe(nh, "/camera/depth/image_rect_raw", 1);
        imu_sub_.subscribe(nh, "/imu/data", 10);

        sync_.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), rgb_sub_, depth_sub_, imu_sub_));
        sync_->registerCallback(boost::bind(&VisualOdometryWithIMU::syncCallback, this, _1, _2, _3));
        odom_pub_ = nh.advertise<nav_msgs::Odometry>("visual_odometry", 10);

        orb_ = cv::ORB::create(2000);
        matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING, true);
    }

private:

   cv::Mat depth_;

    void syncCallback(const sensor_msgs::ImageConstPtr& rgb_msg,
                  const sensor_msgs::ImageConstPtr& depth_msg,
                  const sensor_msgs::ImuConstPtr& imu_msg){
  
        tf2::Quaternion q(
                imu_msg->orientation.x,
                imu_msg->orientation.y,
                imu_msg->orientation.z,
                imu_msg->orientation.w);
        tf2::Matrix3x3(q).getRPY(roll_, pitch_, dummy_yaw_);

        try {
            depth_ = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("Depth cv_bridge exception: %s", e.what());
            return;
        }

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::MONO8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        imageProcessing(cv_ptr->image, rgb_msg->header.stamp);
    }

    void imageProcessing(const cv::Mat& img, const ros::Time& stamp) {

        if (depth_.empty()) {
            ROS_WARN("Depth image is empty. Skipping frame.");
            return;
        }

        // 1. 이전 프레임과 현재 프레임의 특징점 추출
        std::vector<cv::KeyPoint> temp_keypoints;
        cv::Mat temp_descriptors;
        orb_->detectAndCompute(img, cv::noArray(), temp_keypoints, temp_descriptors);
        //ROS_WARN("Initial ORB keypoints: %lu", temp_keypoints.size());
        //ROS_WARN("temp_descriptors empty? %d", temp_descriptors.empty());
        //ROS_WARN("previous_descriptors_ empty? %d", previous_descriptors_.empty());

        if (first_frame_ || previous_descriptors_.empty()) {
            previous_keypoints_ = temp_keypoints;
            previous_descriptors_ = temp_descriptors.clone();  
            previous_image_ = img.clone();
            yaw_ = dummy_yaw_;
            first_frame_ = false;
            return;
        }

        // 2. 전체 매칭 수행
        std::vector<cv::DMatch> matches;
        matcher_->match(previous_descriptors_, temp_descriptors, matches);
        //ROS_WARN("Initial matches: %lu", matches.size());

        cv::Mat motion_mask(img.size(), CV_8UC1, cv::Scalar(0)); //고정된 물체를 기반으로 마스크 생성
        for (const auto& match : matches) {
            const cv::Point2f& pt1 = previous_keypoints_[match.queryIdx].pt;
            const cv::Point2f& pt2 = temp_keypoints[match.trainIdx].pt;
            float dx = pt2.x - pt1.x;
            float dy = pt2.y - pt1.y;
            float motion = std::sqrt(dx * dx + dy * dy);
            //ROS_WARN("motion:%f", motion);
            // 모션 벡터 크기가 작으면 (고정된 배경일 가능성)
            if (motion < 100.0) {  // 100.0 픽셀 이하만 마스크에 포함
                int x = static_cast<int>(pt2.x);
                int y = static_cast<int>(pt2.y);
                if (x >= 0 && x < img.cols && y >= 0 && y < img.rows)
                    cv::circle(motion_mask, cv::Point(x, y), 30, cv::Scalar(255), -1);  // 반지름 30의 영역을 마스크에 추가
            }
        }

        std::vector<cv::KeyPoint> curr_keypoints;
        cv::Mat curr_descriptors;
        orb_->detectAndCompute(img, motion_mask, curr_keypoints, curr_descriptors);
        //ROS_WARN("ORB keypoints after motion mask: %lu", curr_keypoints.size());

        if (curr_descriptors.empty() || previous_descriptors_.empty()) {
            ROS_WARN("Descriptors are empty. Skipping this frame.");
            return;
        }
        if (curr_descriptors.type() != previous_descriptors_.type() ||
            curr_descriptors.cols != previous_descriptors_.cols) {
            ROS_ERROR("Descriptor type or size mismatch!");
            return;
        }

        std::vector<cv::DMatch> raw_matches;
        matcher_->match(previous_descriptors_, curr_descriptors, raw_matches);
        //ROS_WARN("Final matches: %lu", raw_matches.size());

        double fx = 381.36246688113556;
        double fy = 381.36246688113556;
        double cx = 320.5;
        double cy = 240.5;
    

        // 매칭 쌍에서 좌표 추출 (이전 프레임 vs 현재 프레임)
        std::vector<cv::Point2f> pts_prev, pts_curr;
        std::vector<cv::Point3f> pts3d;
        std::vector<cv::Point2f> pts2d;

        for (const auto& match : raw_matches) {
            const cv::Point2f& pt_prev = previous_keypoints_[match.queryIdx].pt;
            const cv::Point2f& pt_curr = curr_keypoints[match.trainIdx].pt;

            int x = static_cast<int>(pt_prev.x);
            int y = static_cast<int>(pt_prev.y);
            float d = depth_.at<uint16_t>(y, x) * 0.001f;

            if (x < 0 || x >= depth_.cols || y < 0 || y >= depth_.rows){
                continue;
            }

            if (d <= 0.001 || d > 2.0){
                continue;
            }

            float X = (x - cx) * d / fx;
            float Y = (y - cy) * d / fy;
            float Z = d;

            pts3d.push_back(cv::Point3f(X, Y, Z));
            pts2d.push_back(pt_curr);
        }

        cv::Mat inlier_mask;
        cv::Mat rvec, tvec;
        cv::Mat K = (cv::Mat_<double>(3, 3) << fx, 0, cx,
                                       0, fy, cy,
                                       0, 0, 1);

        std::vector<cv::Point2f> pts2d_float;
        for (const auto& pt : pts2d){
            pts2d_float.push_back(cv::Point2f(pt.x, pt.y));     
        }

        if (pts3d.size() < 4 || pts2d.size() < 4 || pts3d.size() != pts2d.size()) {
            ROS_WARN("solvePnPRansac: invalid input size (3D=%lu, 2D=%lu)", pts3d.size(), pts2d.size());
            return;
        }
                                
        bool success = cv::solvePnPRansac(
                        pts3d, pts2d_float,         // 3D-2D point correspondences
                        K, cv::Mat(),         // camera matrix, no distortion
                        rvec, tvec,           // output rotation, translation
                        false,                // useExtrinsicGuess
                        100,                  // iterations
                        8.0,                  // reprojection error threshold in pixels
                        0.99,                 // confidence
                        inlier_mask);         // output mask of inliers         


        // RANSAC 결과로 inlier 추출, inlier만 남겨 추정에 사용
        std::vector<cv::Point3f> inlier_pts3d;
        std::vector<cv::Point2f> inlier_pts2d;

        if(success){
           for (size_t i = 0; i < inlier_mask.rows; ++i) {
                if (inlier_mask.at<uchar>(i)) {
                    inlier_pts3d.push_back(pts3d[i]);
                    inlier_pts2d.push_back(pts2d_float[i]);
                }
            } 
        }

        if (inlier_pts2d.size() >= 5) {

            cv::Mat R;
            cv::Rodrigues(rvec, R);

            double dx = tvec.at<double>(0);
            double dy = tvec.at<double>(1);
            double dz = tvec.at<double>(2);

            x_ += dx;
            y_ += dy;

            double imu_yaw_ = dummy_yaw_;
            double dyaw = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0));
            yaw_ += dyaw;

            static int count = 0;
            if (++count % 10 == 0) { //yaw_를 VO로 계속 추정하되, 일정 간격마다 IMU와 VO의 yaw를 비교해 드리프트 보정을 적용합니다.
                double alpha = 0.3; // 0.0 (VO only) ~ 1.0 (IMU only), alpha는 보정 강도입니다.
                double yaw_error = angles::shortest_angular_distance(yaw_, imu_yaw_);
                yaw_ += alpha * yaw_error;
            }
            
            nav_msgs::Odometry odom; //결과 퍼블리시
            odom.header.stamp = stamp;
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
            ROS_WARN("publish continuosly");
            odom_pub_.publish(odom);
        }

        previous_image_ = img.clone(); // 마지막에 저장할 때는 최종 사용된 특징점과 디스크립터로 저장해야 함
        previous_keypoints_ = curr_keypoints;
        previous_descriptors_ = curr_descriptors.clone();
    }

    image_transport::ImageTransport it_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Imu> MySyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub_;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub_;
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


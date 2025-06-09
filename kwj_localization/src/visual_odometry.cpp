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

class VisualOdometryWithIMU {
public:
    VisualOdometryWithIMU(ros::NodeHandle& nh) : it_(nh), first_frame_(true), x_(0.0), y_(0.0), yaw_(0.0),
                                                    roll_(0.0), pitch_(0.0) {
        image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &VisualOdometryWithIMU::imageCallback, this);
        imu_sub_ = nh.subscribe("/imu/data", 10, &VisualOdometryWithIMU::imuCallback, this);
        odom_pub_ = nh.advertise<nav_msgs::Odometry>("visual_odometry", 10);

        orb_ = cv::ORB::create(2000);
        matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING, true);
    }

private:

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);
        tf2::Matrix3x3(q).getRPY(roll_, pitch_, dummy_yaw_);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat img = cv_ptr->image;

        if (first_frame_) {
            orb_->detectAndCompute(img, cv::noArray(), previous_keypoints_, previous_descriptors_);
            previous_image_ = img.clone();
            first_frame_ = false;
            return;
        }

        // 1. 이전 프레임과 현재 프레임의 특징점 추출
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        orb_->detectAndCompute(img, cv::noArray(), keypoints, descriptors);

        // 2. 전체 매칭 수행
        std::vector<cv::DMatch> matches;
        matcher_->match(previous_descriptors_, descriptors, matches);

        cv::Mat motion_mask(img.size(), CV_8UC1, cv::Scalar(0)); //고정된 물체를 기반으로 마스크 생성
        for (const auto& match : matches) {
            const cv::Point2f& pt1 = previous_keypoints_[match.queryIdx].pt;
            const cv::Point2f& pt2 = keypoints[match.trainIdx].pt;
            float dx = pt2.x - pt1.x;
            float dy = pt2.y - pt1.y;
            float motion = std::sqrt(dx * dx + dy * dy);

            // 모션 벡터 크기가 작으면 (고정된 배경일 가능성)
            if (motion < 2.0) {  // 2.0 픽셀 이하만 마스크에 포함
                int x = static_cast<int>(pt2.x);
                int y = static_cast<int>(pt2.y);
                if (x >= 0 && x < img.cols && y >= 0 && y < img.rows)
                    motion_mask.at<uchar>(y, x) = 255;
            }
        }

        orb_->detectAndCompute(img, motion_mask, keypoints, descriptors); //특징점을 다시 추출

        std::vector<cv::DMatch> raw_matches;
        matcher_->match(previous_descriptors_, descriptors, raw_matches); //마스크 기반 특징점으로 정확한 매칭 재수행

        // 매칭 쌍에서 좌표 추출 (이전 프레임 vs 현재 프레임)
        std::vector<cv::Point2f> pts_prev, pts_curr;
        for (const auto& match : raw_matches) {
            pts_prev.push_back(previous_keypoints_[match.queryIdx].pt);
            pts_curr.push_back(keypoints[match.trainIdx].pt);
        }

        std::vector<uchar> ransac_mask;
        // RANSAC으로 inlier 필터링: RANSAC으로 외란 제거
        if (pts_prev.size() >= 5 && pts_curr.size() >= 5) {
            cv::Mat F = cv::findFundamentalMat(pts_prev, pts_curr, cv::FM_RANSAC, 3.0, 0.99, ransac_mask);
        } else {
            //ROS_WARN("Not enough matches for findFundamentalMat (prev: %zu, curr: %zu)", pts_prev.size(), pts_curr.size());
            return;
        }

        // RANSAC 결과로 inlier 추출, inlier만 남겨 추정에 사용
        std::vector<cv::Point2f> inlier_pts_prev, inlier_pts_curr;
        for (size_t i = 0; i < ransac_mask.size(); ++i) {
            if (ransac_mask[i]) {
                inlier_pts_prev.push_back(pts_prev[i]);
                inlier_pts_curr.push_back(pts_curr[i]);
            }
        }

        if (inlier_pts_prev.size() >= 5) {
            cv::Mat mask_pose;
            cv::Mat E = cv::findEssentialMat(inlier_pts_curr, inlier_pts_prev, 700, //매칭된 좌표로 Essential Matrix 추정, inlier들만 사용해 카메라 회전(R)과 이동(t) 계산
                                             cv::Point2d(img.cols / 2, img.rows / 2),
                                             cv::RANSAC, 0.999, 1.0, mask_pose);

            cv::Mat R, t;
            int inliers = cv::recoverPose(E, inlier_pts_curr, inlier_pts_prev, R, t, 700,
                                          cv::Point2d(img.cols / 2, img.rows / 2), mask_pose);

            if (inliers < 10) {
                //ROS_WARN("Too few inliers (%d), skipping this frame", inliers);
                return;
            }

            double dx = t.at<double>(0);
            double dy = t.at<double>(1);

            x_ += dx; //유효한 경우 위치 업데이트
            y_ += dy;

            double imu_yaw_ = dummy_yaw_;
            double dyaw = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0));
            yaw_ += dyaw;

            static int count = 0;
            if (++count % 10 == 0) { //yaw_를 VO로 계속 추정하되, 일정 간격마다 IMU와 VO의 yaw를 비교해 드리프트 보정을 적용합니다.
                double alpha = 0.05; // 0.0 (VO only) ~ 1.0 (IMU only), alpha는 보정 강도입니다.
                double yaw_error = angles::shortest_angular_distance(yaw_, imu_yaw_);
                yaw_ += alpha * yaw_error;
            }
            
            nav_msgs::Odometry odom; //결과 퍼블리시
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
            ROS_WARN("previous publish");
            odom_pub_.publish(odom);
        }

        previous_image_ = img.clone(); //이전 프레임으로 저장
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


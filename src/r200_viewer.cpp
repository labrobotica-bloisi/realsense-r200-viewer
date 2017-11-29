#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

using namespace std;

void rgb_callback(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;

    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::imshow("RGB", cv_ptr->image);
    cv::waitKey(30);
}


void depth_callback(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat depthMat = cv_ptr->image.clone();

    double dmin, dmax;
    cv::minMaxIdx(depthMat, &dmin, &dmax);
    cv::Mat adjMat;
    cv::convertScaleAbs(depthMat, adjMat, 255 / dmax);

    cv::Mat colorMat;
    cv::applyColorMap(adjMat, colorMat, cv::COLORMAP_HOT);
    cv::imshow("Depth", colorMat);

    cv::waitKey(30);

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "realsense_r200_viewer");

    ros::NodeHandle nh;

    string topic_rgb = "camera/rgb/image_rect_color";
    string topic_depth = "camera/depth_registered/sw_registered/image_rect_raw";
	
    cout << "Subscriptions:" << endl;
    cout << "  - RGB topic: " << topic_rgb << endl;
    cout << "  - Depth topic: " << topic_depth << endl;

    ros::Subscriber rgb_sub = nh.subscribe(topic_rgb, 1, &rgb_callback);
    ros::Subscriber depth_sub = nh.subscribe(topic_depth, 1, &depth_callback);

    ros::spin();

    return 0;
}


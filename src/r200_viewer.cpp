

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/thread/thread.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace sensor_msgs;
using namespace message_filters;

using namespace std;

cv::Mat getColorDepth(const cv::Mat& depthImage) {

  cv::Mat colorMat(depthImage.rows, depthImage.cols, CV_8UC3);
  colorMat.setTo(cv::Scalar(0, 0, 0));

  for(int y = 0; y < depthImage.rows; y++) {
    for(int x = 0; x < depthImage.cols; x++) {      
      float d = depthImage.at<float>(y, x);
      if(d != 0 && d < 4000.0f) {
	if(d < 500.f) {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(250, 0, 0);
        }
        else if(d < 750.0f) {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(200, 50, 50);
        }
        else if(d < 1000.0f) {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(150, 100, 100);
        }
        else if(d < 1250.0f) {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(100, 150, 150);
        }
        else if(d < 1500.0f) {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(50, 200, 200);
        }
        else if(d < 1750.0f) {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(0, 250, 250);
        }
        else if(d < 2000.0f) {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(250, 0, 250);
        }
        else if(d < 2250.0f) {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(250, 50, 200);
        }
        else if(d < 2500.0f) {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(200, 100, 150);
        }
        else if(d < 2750.0f) {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(150, 150, 100);
        }
        else if(d < 3000.0f) {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(100, 100, 50);
        }
        else {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(50, 50, 0);
        }
      }
    }
  }
  return colorMat;

}

void callback(const ImageConstPtr& depthImage_, const ImageConstPtr& rgbImage_)
{
    cv_bridge::CvImagePtr rgbImage;
    try {
        rgbImage = cv_bridge::toCvCopy(rgbImage_, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr depthImage;
    try {
        depthImage = cv_bridge::toCvShare(depthImage_, sensor_msgs::image_encodings::TYPE_32FC1);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    cv::Mat colorMat = getColorDepth(depthImage->image);
     
    cv::imshow("Color Depth", colorMat);
    cv::imshow("Depth", depthImage->image);

    cv::waitKey(30);

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "realsense_r200_viewer");

    string topic_rgb = "camera/rgb/image_rect_color";
    string topic_depth = "camera/depth_registered/sw_registered/image_rect_raw";
	
    cout << "Subscriptions:" << endl;
    cout << "  - RGB topic: " << topic_rgb << endl;
    cout << "  - Depth topic: " << topic_depth << endl;

    ros::NodeHandle nh;
    message_filters::Subscriber<Image> depth_sub(nh, topic_depth, 1);
    message_filters::Subscriber<Image> rgb_sub(nh, topic_rgb, 1);

    typedef sync_policies::ApproximateTime<Image, Image> syncPolicy;
    Synchronizer<syncPolicy> sync(syncPolicy(10), depth_sub, rgb_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();

    return 0;
}


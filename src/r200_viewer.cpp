

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
	if(d < 0.8f) {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(0, 0, 191);
        }
	else if(d < 0.9f) {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(0, 0, 255);
        }
        else if(d < 1.0f) {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(0, 63, 255);
        }
        else if(d < 1.2f) {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(0, 127, 255);
        }
        else if(d < 1.4f) {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(0, 191, 255);
        }
        else if(d < 1.6f) {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(0, 255, 255);
        }
        else if(d < 1.8f) {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(63, 255, 191);
        }
        else if(d < 2.0f) {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(127, 255, 127);
        }
        else if(d < 2.2f) {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(191, 255, 63);
        }
        else if(d < 2.4f) {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(255, 255, 0);
        }
        else if(d < 2.6f) {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(255, 191, 0);
        }
        else if(d < 2.8f) {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(255, 127, 0);
        }
        else if(d < 3.0f) {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(255, 63, 0);
        }
        else if(d < 3.2f) {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(255, 0, 0);
        }
        else if(d < 3.4f) {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(191, 0, 0);
        }
        else {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(143, 0, 0);
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
    string topic_depth = "camera/depth_registered/sw_registered/image_rect";
	
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


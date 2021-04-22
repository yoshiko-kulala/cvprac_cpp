#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Point.h>

using namespace cv;
Mat Erode,Dilate,output;
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher cob_point;
  geometry_msgs::Point vel;
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
      &ImageConverter::imageCb, this);
    cob_point = nh_.advertise<geometry_msgs::Point>("/cobo_point", 10);
    //namedWindow(OPENCV_WINDOW);
  }
  ~ImageConverter()
  {
    //destroyWindow(OPENCV_WINDOW);
  }
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    Mat hsv_image,out_image,hand_mask_image,obj_mask_image;
    //imshow("RawImage", cv_ptr->image);
    cvtColor(cv_ptr->image,hsv_image,CV_BGR2HSV,3);

    Scalar hand_min = Scalar(27, 102, 133);
    Scalar hand_max = Scalar(29, 153, 143);
    Scalar obj_min = Scalar(0, 165, 89);
    Scalar obj_max = Scalar(10, 189, 105);
    inRange(hsv_image, hand_min, hand_max,hand_mask_image);
    inRange(hsv_image, obj_min, obj_max,obj_mask_image);
    //imshow("HandMaskImage", hand_mask_image);
    //imshow("ObjectMaskImage", obj_mask_image);

    Moments hand_mu = moments( hand_mask_image, false );
    Point2f hand_mc = Point2f( hand_mu.m10/hand_mu.m00 , hand_mu.m01/hand_mu.m00 );
    circle( cv_ptr->image, hand_mc, 15, Scalar(100), 2, 4);

    Moments obj_mu = moments( obj_mask_image, false );
    Point2f obj_mc = Point2f( obj_mu.m10/obj_mu.m00 , obj_mu.m01/obj_mu.m00 );
    circle( cv_ptr->image, obj_mc, 15, Scalar(100), 2, 4);

    imshow("OutImage", cv_ptr->image);

    vel.x=obj_mc.x-hand_mc.x;
    vel.y=0;
    vel.z=0;
    cob_point.publish(vel);

    waitKey(3);
  }
};
int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
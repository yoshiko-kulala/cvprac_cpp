#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
Mat Erode,Dilate,output;
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher cob_point;
  geometry_msgs::Point vel;
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
      &ImageConverter::imageCb, this);
    cob_point = it_.advertise<geometry_msgs::Point>("cobo_point", 10);
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
    Mat hsv_image,out_image,mask_image;
    imshow("RawImage", cv_ptr->image);
    cvtColor(cv_ptr->image,hsv_image,CV_BGR2HSV,3);
    Scalar s_min = Scalar(4, 127, 102);
    Scalar s_max = Scalar(9, 230, 153);
    inRange(hsv_image, s_min, s_max,mask_image);
    imshow("MaskImage", mask_image);

    Moments mu = moments( mask_image, false );
    Point2f mc = Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );
    circle( cv_ptr->image, mc, 15, Scalar(100), 2, 4);
    imshow("OutImage", cv_ptr->image);

    waitKey(3);
    image_pub_.publish(cv_ptr->toImageMsg());

    vel.x=1;
    vel.y=2;
    vel.y=3;
    cob_point.publish(vel);
  }
};
int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
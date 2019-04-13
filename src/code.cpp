#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
//#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <opencv/cv.h>
#include <image_geometry/pinhole_camera_model.h>
//#include <tf/transform_listener.h>
//#include <boost/foreach.hpp>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber image_sub_;
  //image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  image_geometry::PinholeCameraModel cam_model_;
  ros::Publisher position_pub;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribeCamera("/camera/rgb/image_raw", 10, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 10);
    position_pub = nh_.advertise<geometry_msgs::Point>("/right_hand/position/from_color",10);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg)
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
    cam_model_.fromCameraInfo(info_msg);
    int row = 0; 
    int col = 0; 
    int dis_min = 1000000; 
    for (int i=0; i < cv_ptr->image.rows; i++) {
        for (int j=0; j < cv_ptr->image.cols; j++) {
            int b = cv_ptr->image.at<cv::Vec3b>(i, j).val[0];
            int g = cv_ptr->image.at<cv::Vec3b>(i, j).val[1];
            int r = cv_ptr->image.at<cv::Vec3b>(i, j).val[2]; 
            // int dis = pow((r-255)*(r-255) + g*g + b*b, 0.5); 
            int dis = (r-255)*(r-255) + g*g + b*b; 
            // ROS_INFO("b: %d", b); 
            // ROS_INFO("g: %d", g); 
            // ROS_INFO("r: %d", r); 
            // ROS_INFO("%d", dis); 
            if (dis < dis_min) {
                dis_min = dis; 
                row = i; 
                col = j; 
            }
        }
    }
    cv::Point2d uv = cam_model_.rectifyPoint(cv::Point2d(col,row));
    cv::Point3d xyz = cam_model_.projectPixelTo3dRay(uv);
    geometry_msgs::Point P;
    P.x = xyz.x;
    P.y = xyz.y;
    P.z = xyz.z;
    position_pub.publish(P);
    std::cout << "x: " << P.x << " y: " << P.y << " z: " << P.z << std::endl; 
    cv::circle(cv_ptr->image, cv::Point(col, row), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}

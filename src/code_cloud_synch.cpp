// ROS headers
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/image_encodings.h>

// OpenCV headers
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// c++ headers
#include <math.h>

// ROS synchronization headers
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>


static const std::string OPENCV_WINDOW = "Image window";
using namespace sensor_msgs;
using namespace message_filters;

typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;

class HandLocator
{
public:
  ros::NodeHandle nh;
  //void Cloud_callback(const sensor_msgs::PointCloud2::ConstPtr&,
  //               const sensor_msgs::ImageConstPtr&, tf::TransformBroadcaster&);
  //image_transport::ImageTransport it;
  ros::Publisher image_pub_;
  ros::Publisher position_pub;
  message_filters::Subscriber <sensor_msgs::Image> *image_sub_;
  message_filters::Subscriber <sensor_msgs::PointCloud2> *pc2;
  Synchronizer<MySyncPolicy> *sync;
  cv::Mat image;
  bool displayed;

//public: 
  HandLocator()
    : nh()
  {
    this->displayed = true;
    // Subscrive to input video feed and publish output video feed
    image_sub_ = new message_filters::Subscriber <sensor_msgs::Image>(nh, "/camera/rgb/image_rect_color", 1);
    pc2 = new message_filters::Subscriber <sensor_msgs::PointCloud2>(nh, "/camera/depth_registered/points", 1);
    //typedef sync_policies::ApproximateTime <sensor_msgs::Image,sensor_msgs::PointCloud2> MySyncPolicy;
    sync = new Synchronizer<MySyncPolicy>(MySyncPolicy(3),*pc2, *image_sub_);
    //Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub_,pc2);
    sync->registerCallback(boost::bind(&HandLocator::Cloud_callback, this, _1, _2));
    image_pub_ = nh.advertise<sensor_msgs::Image>("/image_converter/output_video", 10);
    position_pub = nh.advertise<geometry_msgs::PoseStamped>("/right_hand/position/from_color",1);
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~HandLocator()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  //void Cloud_callback(const sensor_msgs::PointCloud2 pCloud)
  void Cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& pCloud,
                      const sensor_msgs::ImageConstPtr& msg)
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
    
      int row = 0; 
      int col = 0; 
      int dis_min = 1000000; 
      for (int i=0; i < cv_ptr->image.rows; i++) {
          for (int j=0; j < cv_ptr->image.cols; j++) {
              int b = cv_ptr->image.at<cv::Vec3b>(i, j).val[0];
              int g = cv_ptr->image.at<cv::Vec3b>(i, j).val[1];
              int r = cv_ptr->image.at<cv::Vec3b>(i, j).val[2]; 
              int dis = (r-255)*(r-255) + g*g + b*b; 
              if (dis < dis_min) {
                  dis_min = dis; 
                  row = i; 
                  col = j; 
              }
          }
      }
      cv::Point2d uv = cv::Point2d(col,row);
      cv::circle(cv_ptr->image, cv::Point(col, row), 10, CV_RGB(255,0,0));

      int height = pCloud->height;
      int width = pCloud->width;
      float X,Y,Z;
      float x,y,z;
      X = x;
      Y = y;
      Z = 1000000;
      int col_1 = col;
      int row_1 = row;
      int kernel = 2;// this is to indicate a kernel of (2n+1)x(2n+1) represented by n
      float dist = 100000;
      float dist_min = 100000;
      for (int i = -kernel; i < kernel+1; i++)
      {
        if (row+i >= height || row+i <0)
        {continue;}
            for (int j = -kernel; j < kernel+1; j++)
            {
            if (col+j >= width || col +j<0)
            {continue;}
            int arrayPosition = (row+i) * pCloud->row_step + (col+j) * pCloud->point_step;
            int arrayPosX = arrayPosition + pCloud->fields[0].offset; // X has an offset of 0
            int arrayPosY = arrayPosition + pCloud->fields[1].offset;// Y has an offset of 4
            int arrayPosZ = arrayPosition + pCloud->fields[2].offset; // Z has an offset of 8
            
            memcpy(&x, &pCloud->data[arrayPosX],  sizeof(float));
            memcpy(&y, &pCloud->data[arrayPosY], sizeof(float));
            memcpy(&z, &pCloud->data[arrayPosZ], sizeof(float));
            
            if (isnan(x) ||isnan(y) ||isnan(z) || abs(z)>15 )
            {continue;}
            dist = z * z + x * x + y * y;
            if (dist < dist_min)
              {
              X = x;
              Y = y;
              Z = z;
              col_1 = col+j;
              row_1 = row+i;
              dist_min = dist;
              }
            }
      }
      geometry_msgs::PoseStamped P;
      P.header.stamp = msg->header.stamp;
      P.header.frame_id = "/camera_rgb_optical_frame";
      P.pose.position.x = X;
      P.pose.position.y = Y;
      P.pose.position.z = Z;
     //static tf::TransformBroadcaster br;
     //tf::Transform transform;
     //transform.setOrigin(tf::Vector3(X,Y,Z));
     //tf::Quaternion q;
     //q.setRPY(0,0,1);
     //transform.setRotation(q);
     //br.sendTransform(tf::StampedTransform(transform, msg->header.stamp,"/camera_depth_frame","right_hand"));
     position_pub.publish(P);
      // Update GUI Window
      //this->image = cv_ptr->image;
      //this->displayed = false;
      cv::circle(cv_ptr->image, cv::Point(col_1, row_1), 10, CV_RGB(0,255,0));
      cv::imshow(OPENCV_WINDOW, cv_ptr->image);
      cv::waitKey(3);

  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hand_locator");
  HandLocator hl;
  while (ros::ok())
	ros::spin();
  return 0;
}

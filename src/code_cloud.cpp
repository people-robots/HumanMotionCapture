#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
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
  image_transport::Subscriber sub_;
  image_transport::Publisher image_pub_;
  image_geometry::PinholeCameraModel cam_model_;
  ros::Publisher position_pub;
  ros::Subscriber pc2;
  int u = 0;
  int v = 0;
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribeCamera("/camera/rgb/image_rect_color", 10, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 10);
    position_pub = nh_.advertise<geometry_msgs::Point>("/right_hand/position/from_color",10);
    pc2 = nh_.subscribe("/camera/depth_registered/points",1, &ImageConverter::Cloud_callback, this);
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }
    
  //void Cloud_callback(const sensor_msgs::PointCloud2 pCloud)
  void Cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& pCloud)
  {
      //int width = pCloud.width;
      //int height = pCloud.height;

      //std::cout << width << ' ' << height << std::endl;
      //int arrayPosition = this->v*pCloud.row_step + this->u*pCloud.point_step;
      //std::cout << this->u << ' ' << this->v << std::endl;
      // compute position in array where x,y,z data start
      //int arrayPosX = arrayPosition + pCloud.fields[0].offset; // X has an offset of 0
      //int arrayPosY = arrayPosition + pCloud.fields[1].offset;
      // Y has an offset of 4
      //int arrayPosZ = arrayPosition +pCloud.fields[2].offset; // Z has an offset of 8
      //int arrayPosrgb = arrayPosition + pCloud.fields[3].offset; // rgb has an offset of 16

      //float X = 0.0;
      //float Y = 0.0;
      //float Z = 0.0;
      //uint24_t rgb = 0;

      //memcpy(&X, &pCloud.data[arrayPosX],  sizeof(float));
      //memcpy(&Y, &pCloud.data[arrayPosY], sizeof(float));
      //memcpy(&Z, &pCloud.data[arrayPosZ], sizeof(float));
      //memcpy(&rgb, &pCloud.data[arrayPosrgb], sizeof(uint32_t));
      // put data into the point p
      //p = geomertry_msgs::Point();
      //p.x = X;
      //p.y = Y;
      //p.z = Z;

      //std::cout<< "cloud " << "x: " << X << " y: " << Y << " z: " << Z << std::endl; 
      //std::cout << "color: " << rgb << std::endl;
      //ROS_INFO("inside callback");
      //
      //
      //std::cout << &pCloud->width << std::endl;
/*
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*pCloud,"x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*pCloud,"y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*pCloud,"z");
      sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_rgb(*pCloud,"rgb");
      //sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_g(*pCloud,"g");
      //sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_b(*pCloud,"b");
      float dist;
      int r,g,b;
      float x,y,z;
      int col,row;
      float min_dist = 200000;
      int height = pCloud->height;
      int width = pCloud->width;
      int r_t,g_t,b_t;
      int t_col,t_row;

      for (size_t i = 0; i < height * width; ++i, ++iter_rgb, ++iter_x, ++iter_y, ++iter_z)
      {
        t_row = int(i/ width);
        t_col = i % width;
        r = (int((*iter_rgb >> 16) & 0x00001f));//*255/32;
        g = int((*iter_rgb >> 8) & 0x00003f);//*255/64;
        b = int(*iter_rgb & 0x00001f);//*255/32;
        dist = r*r + g*g + b*b;
        //if (dist <0)
        //  dist = -dist;
        if (isnan(*iter_x) ||isnan(*iter_y) ||isnan(*iter_z) )
        {continue;}
        //std::cout << this->u << ' '<< this->v << ' ' << t_row << ' ' << t_col << std::endl;
        if (t_row > this->u-1 && t_row < this->u+1 && t_col > this->v-1 && t_col < this->v+1)
        std::cout << *iter_rgb << ' '  << t_row << ' ' << t_col << ' ' << *iter_x << ' ' << *iter_y << ' ' << *iter_z<< std::endl;
        //std::cout << t_row << ' ' << t_col << std::endl;
        if (dist < min_dist )
        {
            //std::cout << t_col << ' ' << t_row << ' ' << r << ' ' << g << ' ' << b << std::endl;
            min_dist = dist;
            x = *iter_x;
            y = *iter_y;
            z = *iter_z;
            row = int(i / width);
            col = i % width;
            //r_t = r;
            //g_t = g;
            //b_t = b;
        }
      }

      */
      int height = pCloud->height;
      int width = pCloud->width;
      float X,Y,Z;
      float x,y,z;
      X = x;
      Y = y;
      Z = 1000000;
      int col_1 = this->u;
      int row_1 = this->v;
      int col = this->u;
      int row = this->v;
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
            
            if (isnan(x) ||isnan(y) ||isnan(z) || z>15 )
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
        this->u = row_1;
        this->v = col_1;
        geometry_msgs::Point P;
        P.x = X;
        P.y = Y;
        P.z = Z;
        position_pub.publish(P);



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
    //cv::Point2d uv = cam_model_.rectifyPoint(cv::Point2d(col,row));
    cv::Point2d uv = cv::Point2d(col,row);
    this->u = int(uv.x);
    this->v = int(uv.y);
    //cv::Point3d xyz = cam_model_.projectPixelTo3dRay(uv);
    //geometry_msgs::Point P;
    //P.x = xyz.x;
    //P.y = xyz.y;
    //P.z = xyz.z;
    //position_pub.publish(P);
    //std::cout << "x: " << P.x << " y: " << P.y << " z: " << P.z << std::endl;
    cv::circle(cv_ptr->image, cv::Point(this->u, this->v), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}

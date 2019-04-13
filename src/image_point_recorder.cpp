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
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sstream>
#include <iostream>
#include <fstream>

using namespace sensor_msgs;
using namespace message_filters;
using namespace std;

typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;

class ImageSaver
{
private:
  ros::NodeHandle nh;
  message_filters::Subscriber <sensor_msgs::Image> *image_sub_;
  message_filters::Subscriber <sensor_msgs::PointCloud2> *pc2;
  Synchronizer<MySyncPolicy> *sync;
  std::string folder;
  bool button_stat;
  std::string old_folder;
  std::string meta_data;
  ros::Subscriber folder_sub;// = nh.subscribe("/target_loc",1,&ImageSaver::target_callback,this);
  ros::Subscriber button_sub;
  int count;
public: 
  ImageSaver()
    : nh()
  {
    this->count = 0;
    this->meta_data = "";
    this->old_folder=std::string(" ");
    this->button_stat = false;
    this->folder_sub = this->nh.subscribe("/target_loc",10,&ImageSaver::target_callback,this);
    this->button_sub = this->nh.subscribe("/button",10,&ImageSaver::button_callback,this);
    // Subscrive to input video feed and publish output video feed
    image_sub_ = new message_filters::Subscriber <sensor_msgs::Image>(nh, "/camera/rgb/image_rect_color", 1);
    pc2 = new message_filters::Subscriber <sensor_msgs::PointCloud2>(nh, "/camera/depth_registered/points", 1);
    //typedef sync_policies::ApproximateTime <sensor_msgs::Image,sensor_msgs::PointCloud2> MySyncPolicy;
    sync = new Synchronizer<MySyncPolicy>(MySyncPolicy(3),*pc2, *image_sub_);
    //Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub_,pc2);
    sync->registerCallback(boost::bind(&ImageSaver::Cloud_callback, this, _1, _2));
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
      if (this->old_folder.compare(" "))
      {
      std::ostringstream ss;
      ss << this->count;
      std::ostringstream stamp;
      stamp << msg->header.stamp.sec ;
      stamp << ".";
      stamp <<  msg->header.stamp.nsec;
      stamp << ", button_state: ";
      stamp << this->button_stat;
      meta_data += "Image_" + ss.str() + ": " + stamp.str() + '\n';
      std::string file_name = this->folder + "/Image_" + ss.str() + ".jpg";
      cv::imwrite(file_name,cv_ptr->image);
      //std::cout << " saving_image" << std::endl;
      file_name = this->folder + "/Cloud_" + ss.str() + ".txt";
      ofstream cloud_file;
      cloud_file.open( file_name.c_str() , ios::binary);
      //cloud_file << "width: ";
      //cloud_file << pCloud->width;
      //cloud_file << '\n';
      //cloud_file << "height: ";
      //cloud_file << pCloud->height;
      //cloud_file << '\n';
      //cloud_file << "stamp: ";
      //cloud_file << '\n';
      //cloud_file << '\t';
      //cloud_file << pCloud->header.stamp;
      //cloud_file << '\n';
      //cloud_file << "data: ";
      //char *a = &pCloud->data[0];
      cloud_file.write(reinterpret_cast<const char*>(&pCloud->data[0]),640*480*4*sizeof(pCloud->data[0]));
      //cloud_file << '\n';
      cloud_file.close();
      this->count ++;
      
      //std::cout << " saving_cloud" << std::endl;
      }
  }

  void target_callback(const std_msgs::String::ConstPtr& st)
  {
    //std::cout << "hello" << std::endl;
    this->folder = st->data;
    //std::cout << this->old_folder << '\n' << this->folder << this->old_folder.compare(this->folder) << std::endl;
    if (this->old_folder.compare(this->folder))
    {
        this->count = 0;
        ofstream data_file;
        std::string file_name = this->old_folder +"/meta_data.txt"; 
        data_file.open(file_name.c_str());
        data_file  << this->meta_data;
        data_file.close();
        this->meta_data = "";
        this->old_folder = this->folder;
        
    }
  }

  void button_callback(const std_msgs::Bool::ConstPtr& data)
  {
    this->button_stat = data->data;

  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "point_image_recorder");
  ImageSaver is;
  ros::spin();
  return 0;
}

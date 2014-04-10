#include "my_fusion/fusion.h"

using namespace sensor_msgs;
using namespace message_filters;

image_transport::Publisher pub_worldmap;

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "fusion");

//   ros::NodeHandle nh;
//   std::sub_lidar_topic_name = std::string("interpreter/obstacleMap/") + std::string(argv[1]);
//   image_transport::ImageTransport image_transporter(nh);
//   pub_worldmap = image_transporter.advertise("/world_map",10);

//   message_filters::Subscriber<Image> image_sub(nh, "sensors/camera/", 1);
//   message_filters::Subscriber<Image> lidar_sub(nh, sub_lidar_topic_name, 1);
//   TimeSynchronizer<Image, Image> sync(image_sub, lidar_sub, 10);

//   sync.registerCallback(boost::bind(&callback, _1, _2));
//   ros::spin();
//   return 0;
// }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fusion");

  ros::NodeHandle nh;
  std::string sub_lidar_topic_name = std::string("interpreter/obstacleMap/") + std::string(argv[1]);
  image_transport::ImageTransport image_transporter(nh);
  pub_worldmap = image_transporter.advertise("/world_map",1000);
  
  ros::Subscriber lidar_sub = nh.subscribe(sub_lidar_topic_name,1000,callback);
  // message_filters::Subscriber<Image> image_sub(nh, "sensors/camera/", 1);
  // message_filters::Subscriber<Image> lidar_sub(nh, sub_lidar_topic_name, 1);
  // TimeSynchronizer<Image, Image> sync(image_sub, lidar_sub, 10);

  // sync.registerCallback(boost::bind(&callback, _1, _2));
  while(ros::ok()){

    ros::spinOnce();

  }
   return 0;
}
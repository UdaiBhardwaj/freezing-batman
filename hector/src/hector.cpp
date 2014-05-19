#include <cassert>
#include <climits>
#include <opencv2/highgui/highgui_c.h>
#include <stdexcept>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>


int map_size = 1000;
int count_receive=0;

void imageCallBack(cv_bridge::CvImage msg)

{	
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<sensor_msgs::LaserScan>("gps", 10);
	ros::Rate loop_rate(10);
	cv::namedWindow("listening", CV_WINDOW_AUTOSIZE);
  	cv::imshow("listening", msg.image);
  	cv::waitKey(10);
  	ROS_INFO("Listened for the %d time\n", count_receive++);
	

	sensor_msgs::LaserScan scan;
	scan.header.seq = 30555;
	scan.header.stamp = ros::Time::now(); //1396780076.43;//1.3967800764300000667572021484375e+9;  //1396780076.43 //(1396779657 + 419430000/1000000);
	scan.header.frame_id = "laser";
	scan.angle_min = -1.57079637051;
	scan.angle_max = 1.56643295288;
	scan.angle_increment = 0.00436332309619;
	scan.time_increment = 1.73611115315e-05;
	scan.scan_time = 0.0250000003725;
	scan.range_min =  0.0230000000447;
	scan.range_max = 60.0;
	int size = (int)((scan.angle_max - scan.angle_min)/scan.angle_increment);
	scan.ranges.resize(size);

	int lidar_y_shift = 30;
	int center_x = 500, center_y = 100;
	int min_dist = 0, max_dist = 400;

	for(int i = msg.image.rows; i>=0; --i)
		for(int j=0; j<msg.image.cols; ++j)
			if(msg.image.at<uchar>(i, j) == 255)
			{
				int x1 = j;
				int y1 = map_size - i - 30 - 1;
				if(y1 > min_dist && y1 < max_dist)
				{
					double x2 = (x1 - center_x)/100;
					double y2 = (y1 - center_y - lidar_y_shift)/100;
					float dist = sqrt( pow(x2, 2) + pow(y2, 2) );
					float angle = atan2(y2, -x2);
					int counter = (int)((angle - scan.angle_min)/scan.angle_increment);
					if(dist>scan.range_min && dist<scan.range_max)
						scan.ranges[counter] = dist;
				}
			}

	int count = 0;
	while(ros::ok())
	{
		pub.publish(scan);
		std::cout<<"Sending scan for the "<<count<<" time\n";

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
	}

}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "hector");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("chatter", 10, imageCallBack);
	ros::spin();
	return 0;
}
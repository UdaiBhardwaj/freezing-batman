#include <image_transport/image_transport.h>

#include <ros/ros.h>

#include <stdexcept>
#include <iostream>
#include <fstream>
#include <sensor_msgs/Image.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

char NORMAL[]= "\033[0m";
char RED[]= "\033[0;31m";
char BLUE[]= "\033[0;34m";
char WHITE[]= "\033[37;01m";

std::ofstream commonFile;

cv::Mat img,temp, binary;

std::string name;

int size_sample=8;

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	int countP,countN;
	cv::Mat imgRoi;
     if  ( event == cv::EVENT_LBUTTONDOWN ){
          std::cout << BLUE << "Saving positive sample - position (" << x << ", " << y << ")" << NORMAL << std::endl;
          imgRoi = temp(cv::Rect(x-size_sample/2, y-size_sample/2, size_sample, size_sample));
          
          commonFile<<"+1 ";
          countP=0;
          for(int i=0;i<size_sample;i++){
			  for(int j=0;j<size_sample;j++){
				  commonFile<<countP++<<":"<<(int)imgRoi.at<cv::Vec3b>(i,j)[0]<<" ";
				  commonFile<<countP++<<":"<<(int)imgRoi.at<cv::Vec3b>(i,j)[1]<<" ";
				  commonFile<<countP++<<":"<<(int)imgRoi.at<cv::Vec3b>(i,j)[2]<<" ";
			  }
		  }
          
          commonFile<<countP++<<":-1"<<std::endl;
          cv::rectangle( img, cv::Point( x-size_sample/2, y-size_sample/2 ), cv::Point( x+size_sample/2, y+size_sample/2 ), cv::Scalar( 255, 255, 255 ));
     }
     else if  ( event == cv::EVENT_RBUTTONDOWN ){
		 std::cout << RED << "Saving negative sample - position (" << x << ", " << y << ")" << NORMAL << std::endl;
		 imgRoi = temp(cv::Rect(x-size_sample/2, y-size_sample/2, size_sample, size_sample));
		 
		 commonFile<<"-1 ";
		 countN=0;
          for(int i=0;i<size_sample;i++){
			  for(int j=0;j<size_sample;j++){
				commonFile<<countN++<<":"<<(int)imgRoi.at<cv::Vec3b>(i,j)[0]<<" ";
				commonFile<<countN++<<":"<<(int)imgRoi.at<cv::Vec3b>(i,j)[1]<<" ";
				commonFile<<countN++<<":"<<(int)imgRoi.at<cv::Vec3b>(i,j)[2]<<" ";
			  }
		  }
          
          commonFile<<countN++<<":-1"<<std::endl;
          cv::rectangle( img, cv::Point( x-size_sample/2, y-size_sample/2 ), cv::Point( x+size_sample/2, y+size_sample/2 ), cv::Scalar( 0, 0, 0 ));
     }
}


void CallBackFunc2(int event, int x, int y)
{
	int countP,countN;
	//cv::Mat imgRoi;
     if  ( event == 1){
          std::cout << BLUE << "Saving positive sample - position (" << x << ", " << y << ")" << NORMAL << std::endl;
         // imgRoi = temp(cv::Rect(x-size_sample/2, y-size_sample/2, size_sample, size_sample));
          
          commonFile<<"+1 ";
          countP=0;
          for(int i=y - size_sample/2;i< y+ size_sample/2; i++){
			  for(int j= x - size_sample/2 ;j< x + size_sample/2 ;j++){
				  commonFile<<countP++<<":"<<(int)img.at<cv::Vec3b>(i,j)[0]<<" ";
				  commonFile<<countP++<<":"<<(int)img.at<cv::Vec3b>(i,j)[1]<<" ";
				  commonFile<<countP++<<":"<<(int)img.at<cv::Vec3b>(i,j)[2]<<" ";
			  }
		  }
          
          commonFile<<countP++<<":-1"<<std::endl;
          //cv::rectangle( img, cv::Point( x-size_sample/2, y-size_sample/2 ), cv::Point( x+size_sample/2, y+size_sample/2 ), cv::Scalar( 255, 255, 255 ));
     }
     else if  ( event == -1){
		 std::cout << RED << "Saving negative sample - position (" << x << ", " << y << ")" << NORMAL << std::endl;
		 //imgRoi = temp(cv::Rect(x-size_sample/2, y-size_sample/2, size_sample, size_sample));
		 
		 commonFile<<"-1 ";
		 countN=0;
          for(int i=y - size_sample/2;i< y+ size_sample/2; i++){
			  for(int j= x - size_sample/2 ;j< x + size_sample/2 ;j++){
				commonFile<<countN++<<":"<<(int)img.at<cv::Vec3b>(i,j)[0]<<" ";
				commonFile<<countN++<<":"<<(int)img.at<cv::Vec3b>(i,j)[1]<<" ";
				commonFile<<countN++<<":"<<(int)img.at<cv::Vec3b>(i,j)[2]<<" ";
			  }
		  }
          
          commonFile<<countN++<<":-1"<<std::endl;
          //cv::rectangle( img, cv::Point( x-size_sample/2, y-size_sample/2 ), cv::Point( x+size_sample/2, y+size_sample/2 ), cv::Scalar( 0, 0, 0 ));
     }
}





int main(int argc, char** argv) {
	if(argc<2){
		printf("Usage: <name> <image_file> <optional:= kernel_size>\n");
		return -1;
	}
	commonFile.open("Samples2",std::fstream::app);
	img=cv::imread(argv[1]);
	binary = cv::imread("/home/udai/fuerte_workspace/sandbox/freezing-batman/environment/interpretation/lane_detector_test/lanebinary.jpg");
	
	if(( argc == 3 )||(argc == 4)){
		int temp_size_sample = atoi(argv[2]);
		if( temp_size_sample>=5 && temp_size_sample<=30){
			size_sample = temp_size_sample;
		}
	}
	printf("Sample Size: %d\n",size_sample);

	temp=img.clone();
	if ( img.empty() ){ 
          std::cout << "Error loading the image" << std::endl;
          return -2; 
    }
	cv::namedWindow("Original Image");
	int choice = 0;
	if(argc == 4) choice = atoi(argv[3]);
	if(choice==1)
	{	std::cout<<"Dynamic input\n";
		int square_size = 80;
		for(int i = img.rows/2 - square_size/2 + size_sample/2; i<(img.rows/2 + square_size/2 - size_sample/2); i+=size_sample)
			for(int j = img.cols/2 - square_size/2 + size_sample/2; j<(img.cols/2 + square_size/2 - size_sample/2); j+=size_sample)
			{
				CallBackFunc2(1, j, i);
			}
		/*
		for(int i = img.rows/2 - square_size/2; i< img.rows/2 + square_size/2; ++i)
			for(int j = img.cols/2 - square_size/2; j<img.cols/2 + square_size/2; ++j)
				CallBackFunc2(1, j, i);*/

		for(int k = 0; k < binary.rows; ++k)
			for(int l = 0; l<binary.cols; ++l)
			{
				int intensity = binary.at<uchar>(k, l);
				if(intensity>240)
					CallBackFunc2(-1, l, k);
			}
		std::cout<<"Exiting dynamic input\n";
	}
	else
		cv::setMouseCallback("Original Image", CallBackFunc, NULL);
	while(true){
		cv::imshow("Original Image",img);
		cv::waitKey(10);
	}
    return 0;
}

#include <ros/ros.h>  
#include <image_transport/image_transport.h>  
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>  

using namespace std;
using namespace cv;

char flag = 0;
VideoWriter writer;

void imageCallback(const sensor_msgs::ImageConstPtr &cam_image)
{
cv_bridge::CvImagePtr cv_ptr;
try
{
   cv_ptr = cv_bridge::toCvCopy(cam_image,sensor_msgs::image_encodings::BGR8);
 
}

catch (cv_bridge::Exception& e)
{
   ROS_ERROR("cv_bridge exception:%s",e.what());
   return;
}

   Mat img_rgb = cv_ptr->image;
  // imshow("view",img_rgb);
  // cvWaitKey(1);

if (flag == 0)
{
    writer << img_rgb; 
}

if((char(waitKey(1)) == 'q')) 
{
    //cvReleaseVideoWriter(&video);
    flag = 1;
}

}

int main(int argc, char **argv)  
{  
   ros::init(argc, argv, "video_node");  
   ros::NodeHandle nh("~");  
   image_transport::ImageTransport it(nh);  
   image_transport::Subscriber sub = it.subscribe("/aruco_single/result", 1, imageCallback);

   string filename;
   nh.param("filename",filename,std::string("/home/odroid/Videos/tag_detection.avi"));
   //string arg=argv[1];

   //char filename[20]="";   //保存文件名的字符串数组
   //sprintf(filename,"/home/odroid/Videos/video_%d.avi",atoi(arg.c_str()));
   writer.open(filename, CV_FOURCC('M', 'J', 'P', 'G'), 30, Size(640, 480)); 

  // cv::namedWindow("view"); 
   ros::spin();  
   return 0; 
} 

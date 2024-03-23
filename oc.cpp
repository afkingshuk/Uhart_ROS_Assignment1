#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";
using namespace cv;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher color_pub = n.advertise<std_msgs::Int32>("color_topic", 10); //kingshuk

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
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

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(cv_ptr->image.cols/2, cv_ptr->image.rows/2), 10, CV_RGB(0,0,255));
      Vec3b intensity = cv_ptr->image.at<Vec3b>(320, 240);
      float blue = (float)intensity.val[0];
      float green = (float)intensity.val[1];
      float red = (float)intensity.val[2];

      ROS_INFO("blue: %f", blue);
      ROS_INFO("green: %f", green);
      ROS_INFO("red: %f", red);
	  
	  
	  //Create and add std_msgs/Int32 publisher
	  std_msgs::Int32 color; // Create a message object of type Int32
	  
	  //multiple if/3 statement which divide color space into red, green, and blue color objects
	  // Assuming intensity.val[0] represents blue, intensity.val[1] represents green, and intensity.val[2] represents red
	  if (blue > red && blue > green) {
	  	ROS_INFO("The pixel represents a blue color.");
		color.data = 0
	  } else if (green > red && green > blue) {
	  	ROS_INFO("The pixel represents a green color.");
		color.data = 1
	  } else if (red > blue && red > green) {
	  	ROS_INFO("The pixel represents a red color.");
		
	  } else {
	  	ROS_INFO("The pixel represents undefined color.");
		color.data = 99		
	  }
	  	  
	  //publish number => blue 0 green 1 red 2 
	  color_pub.publish(color);

	  
	  

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

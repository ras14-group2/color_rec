#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include <sstream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <robo_globals.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class imageconverter
{

	ros::NodeHandle n_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

private:

public:

	cv::Mat hsv, imgThresholded;
	int iLowH;
	int iHighH;

	int iLowS;
	int iHighS;

	int iLowV;
	int iHighV;

	imageconverter()
	: it_(n_), iLowH(0), iHighH(179), iLowS(0), iHighS(255), iLowV(0), iHighV(255)
	{

	// Subscribe to input video feed and publish output video feed
			image_sub_ = it_.subscribe("camera/rgb/image_raw", 1, &imageconverter::imageCb, this); // /camera/image_raw
			image_pub_ = it_.advertise("/image_converter/output_video", 1);

	cv::namedWindow(OPENCV_WINDOW);

	cv::namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

	//Create trackbars in "Control" window
	cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
	cvCreateTrackbar("HighH", "Control", &iHighH, 179);

	cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS", "Control", &iHighS, 255);

	cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
	cvCreateTrackbar("HighV", "Control", &iHighV, 255);
}

	~imageconverter()
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

		cvtColor(cv_ptr->image, hsv, CV_BGR2HSV);

		inRange(hsv, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

		//morphological opening (removes small objects from the foreground)
		erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
		dilate( imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

		//morphological closing (fill small holes in the foreground)
		dilate( imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
		erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

		// Update GUI Window
		cv::imshow(OPENCV_WINDOW, cv_ptr->image);
		cv::imshow("HSV",hsv);
		cv::imshow("Image Thresholed",imgThresholded);
		cv::waitKey(3);

		// Output modified video stream
		image_pub_.publish(cv_ptr->toImageMsg());
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_converter");
	imageconverter imageconverter_node;
	ros::Rate loop_rate(CTRL_FREQ);

  while (ros::ok())
	{
		ros::spinOnce();
		//imageconverter_node.publish();
		loop_rate.sleep();
  }

  return 0;
}

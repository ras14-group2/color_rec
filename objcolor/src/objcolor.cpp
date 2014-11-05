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

class objcolor
{

	ros::NodeHandle n_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

private:

public:

	cv::Mat hsv, imgThresholded[2];
	int iLowH;
	int iHighH;

	int iLowS;
	int iHighS;

	int iLowV;
	int iHighV;

	cv::Moments oMoments[2];

	double dM01[2];
	double dM10[2];
	double dArea[2];

	int posX[2];
	int posY[2];

	objcolor()
	: it_(n_), iLowH(0), iHighH(179), iLowS(0), iHighS(255), iLowV(0), iHighV(255)
	{


// Subscribe to input video feed and publish output video feed
			image_sub_ = it_.subscribe("camera/rgb/image_raw", 1, &objcolor::imageCb, this); // /camera/image_raw
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

	~objcolor()
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

		//inRange(hsv, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

		inRange(hsv, cv::Scalar(0, 159, 190), cv::Scalar(22, 255, 255), imgThresholded[0]); //Threshold the image
		inRange(hsv, cv::Scalar(40, 110, 80), cv::Scalar(60, 255, 140), imgThresholded[1]); //Threshold the image

		//Calculate the moments of the thresholded image
		oMoments[0] = moments(imgThresholded[0]);
		oMoments[1] = moments(imgThresholded[1]);

		dM01[0] = oMoments[0].m01;
		dM10[0] = oMoments[0].m10;
		dArea[0] = oMoments[0].m00;

		dM01[1] = oMoments[1].m01;
		dM10[1] = oMoments[1].m10;
		dArea[1] = oMoments[1].m00;

		if (dArea[0] > 10000 || dArea[1] > 10000)
		 {
			//calculate the position of the object
			posX[0] = dM10[0] / dArea[0];
			posY[0] = dM01[0] / dArea[0];
			posX[1] = dM10[1] / dArea[1];
			posY[1] = dM01[1] / dArea[1];

			if (posX[1] >= 0 && posY[1] >= 0)
			{
				// Draw an example circle on the video stream
				if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60){
					cv::circle(cv_ptr->image, cv::Point(posX[0], posY[0]), 10, CV_RGB(255,0,0));
					cv::circle(cv_ptr->image, cv::Point(posX[1], posY[1]), 10, CV_RGB(0,0,255));
				}
			}

		 }



		// Update GUI Window
		cv::imshow(OPENCV_WINDOW, cv_ptr->image);
		cv::imshow("HSV",hsv);
		cv::imshow("Image Thresholed",imgThresholded[0]);
		cv::waitKey(3);

		// Output modified video stream
		image_pub_.publish(cv_ptr->toImageMsg());
	}



};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_converter");
	objcolor objcolor_node;
	ros::Rate loop_rate(CTRL_FREQ);

  while (ros::ok())
	{
		ros::spinOnce();
		//objcolor_node.publish();
		loop_rate.sleep();
  }

  return 0;
}

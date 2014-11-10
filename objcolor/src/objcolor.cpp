#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
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

	cv::Mat hsv, imgThresholded[8];
	int iLowH;
	int iHighH;

	int iLowS;
	int iHighS;

	int iLowV;
	int iHighV;

	cv::Moments oMoments[8];

	double dM01[8];
	double dM10[8];
	double dArea[8];

	int posX[8];
	int posY[8];

	objcolor()
	: it_(n_), iLowH(0), iHighH(179), iLowS(0), iHighS(255), iLowV(0), iHighV(255)
	{
		// Subscribe to input video feed and publish output video feed
		image_sub_ = it_.subscribe("camera/rgb/image_raw", 1, &objcolor::imageCb, this); // /camera/image_raw
		image_pub_ = it_.advertise("/image_converter/output_video", 1);

		cv::namedWindow(OPENCV_WINDOW);
	}

	~objcolor()
	{
		cv::destroyWindow(OPENCV_WINDOW);
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		static const std::string color[] = {"Yellow", "Orange", "Light Green", "Green", "Light Blue", "Blue", "Purple", "Red"}; // Colors;

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


		inRange(hsv, cv::Scalar(17, 140, 200), cv::Scalar(38, 255, 255), imgThresholded[0]); //Threshold the image Yellow
		inRange(hsv, cv::Scalar(0, 130, 200), cv::Scalar(16, 255, 255), imgThresholded[1]); //Threshold the image Orange
		inRange(hsv, cv::Scalar(30, 170, 142), cv::Scalar(50, 255, 255), imgThresholded[2]); //Threshold the image L Green
		inRange(hsv, cv::Scalar(40, 110, 80), cv::Scalar(75, 255, 140), imgThresholded[3]); //Threshold the image Green
		inRange(hsv, cv::Scalar(90, 110, 150), cv::Scalar(110, 230, 230), imgThresholded[4]); //Threshold the image L Blue
		inRange(hsv, cv::Scalar(105, 100, 80), cv::Scalar(115, 180, 165), imgThresholded[5]); //Threshold the image Blue
		inRange(hsv, cv::Scalar(120, 90, 80), cv::Scalar(160, 255, 255), imgThresholded[6]); //Threshold the image Purple
		inRange(hsv, cv::Scalar(169, 255, 110), cv::Scalar(179, 255, 255), imgThresholded[7]); //Threshold the image Red

		for(int i=0; i<8 ; i++){

			//morphological opening (removes small objects from the foreground)
			erode(imgThresholded[i], imgThresholded[i], getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
			dilate( imgThresholded[i], imgThresholded[i], getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

			//morphological closing (fill small holes in the foreground)
			dilate( imgThresholded[i], imgThresholded[i], getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
			erode(imgThresholded[i], imgThresholded[i], getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

			//Calculate the moments of the thresholded image
			oMoments[i] = moments(imgThresholded[i]);

			dM01[i] = oMoments[i].m01;
			dM10[i] = oMoments[i].m10;
			dArea[i] = oMoments[i].m00;

			if (dArea[i] > 10000)
			{
				//calculate the position of the object
				posX[i] = dM10[i] / dArea[i];
				posY[i] = dM01[i] / dArea[i];


				if (posX[i] >= 0 && posY[i] >= 0)
				{
					// Draw an example circle on the video stream
					if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
					{
						cv::circle(cv_ptr->image, cv::Point(posX[i], posY[i]), 10, CV_RGB(255,0,0));
						ROS_INFO("Color detected = %s \n", color[i].c_str());
					}
				}
			}
		}

		// Update GUI Window
		cv::imshow(OPENCV_WINDOW, cv_ptr->image);
		cv::imshow("HSV",hsv);
		//cv::imshow("Image Thresholed",imgThresholded[0]);
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
		loop_rate.sleep();
  }

  return 0;
}

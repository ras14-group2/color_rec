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
#include <cmath>
#include <iostream>

static const std::string OPENCV_WINDOW = "Image window";

/**
 * Helper function to find a cosine of angle between vectors
 * from pt0->pt1 and pt0->pt2
 */
static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0)
{
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

/**
 * Helper function to display text in the center of a contour
 */
void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour)
{
	int fontface = cv::FONT_HERSHEY_SIMPLEX;
	double scale = 0.4;
	int thickness = 1;
	int baseline = 0;

	cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
	cv::Rect r = cv::boundingRect(contour);

	cv::Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
	cv::rectangle(im, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255,255,255), CV_FILLED);
	cv::putText(im, label, pt, fontface, scale, CV_RGB(0,0,0), thickness, 8);
}


class objshape
{

	ros::NodeHandle n_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

private:

public:

	cv::Mat hsv, imgThresholded[8],imgThresholded_new[8], dst;
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

	int count;

	std::string obj;

	objshape()
	: it_(n_), iLowH(0), iHighH(179), iLowS(0), iHighS(255), iLowV(0), iHighV(255), count(0)
	{
		// Subscribe to input video feed and publish output video feed
		image_sub_ = it_.subscribe("camera/rgb/image_raw", 1, &objshape::imageCb, this); // /camera/image_raw
		image_pub_ = it_.advertise("/image_converter/output_video", 1);

		cv::namedWindow(OPENCV_WINDOW);
	}

	~objshape()
	{
		cv::destroyWindow(OPENCV_WINDOW);
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		static const std::string color[] = {"Yellow", "Orange", "Light Green", "Green", "Blue", "Blue", "Purple", "Red"}; // Colors;
		static const std::string shape[] = {"Cube", "Ball", "Cylinder", "Triangle", "Patric", "Cross"}; // Shapes;

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

		inRange(hsv, cv::Scalar(17, 140, 200), cv::Scalar(38, 255, 255), imgThresholded_new[0]); //Threshold the image Yellow
		inRange(hsv, cv::Scalar(0, 170, 170), cv::Scalar(10, 255, 255), imgThresholded_new[1]); //Threshold the image Orange
		inRange(hsv, cv::Scalar(30, 170, 142), cv::Scalar(50, 255, 255), imgThresholded_new[2]); //Threshold the image L Green
		inRange(hsv, cv::Scalar(40, 110, 80), cv::Scalar(75, 255, 140), imgThresholded_new[3]); //Threshold the image Green
		inRange(hsv, cv::Scalar(90, 110, 150), cv::Scalar(110, 230, 230), imgThresholded_new[4]); //Threshold the image L Blue
		inRange(hsv, cv::Scalar(105, 100, 80), cv::Scalar(115, 180, 165), imgThresholded_new[5]); //Threshold the image Blue
		inRange(hsv, cv::Scalar(120, 90, 80), cv::Scalar(160, 255, 255), imgThresholded_new[6]); //Threshold the image Purple
		inRange(hsv, cv::Scalar(160, 150, 110), cv::Scalar(179, 255, 255), imgThresholded_new[7]); //Threshold the image Red

		if(count < 6)
		{
			for(int i=0; i<8 ; i++)
			{
				if(count==0)
				{
					imgThresholded[i] = imgThresholded_new[i];
				}else{
					cv::add(imgThresholded_new[i], imgThresholded[i], imgThresholded[i]);
				}
			}
			count++;
		}

		dst = cv_ptr->image.clone();

		if(count==6)
		{
			count = 0;
			for(int i=0; i<8 ; i++){

				//morphological opening (removes small objects from the foreground)
				erode(imgThresholded[i], imgThresholded[i], getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)) );
				dilate( imgThresholded[i], imgThresholded[i], getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)) );

				//morphological closing (fill small holes in the foreground)
				dilate( imgThresholded[i], imgThresholded[i], getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)) );
				erode(imgThresholded[i], imgThresholded[i], getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)) );

				std::vector<cv::Vec4i> hierarchy;
				// Find contours
				std::vector<std::vector<cv::Point> > contours;
				cv::findContours(imgThresholded[i].clone() , contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

				std::vector<cv::Point> approx;


				for (int k = 0; k < contours.size(); k++)
				{
					obj = "";
					// Approximate contour with accuracy proportional
					// to the contour perimeter
					cv::approxPolyDP(cv::Mat(contours[k]), approx, cv::arcLength(cv::Mat(contours[k]), true)*0.02, true);

					// Skip small or non-convex objects
					if (std::fabs(cv::contourArea(contours[k])) < 1000 || !cv::isContourConvex(approx))
					{
						//						if(!cv::isContourConvex(approx))
						//						{
						//							ROS_INFO("non convex object detected");
						//						}
						continue;
					}

					// iterate through all the top-level contours,
					// draw each connected component with its own random color
					int idx = 0;
					for( ; idx >= 0; idx = hierarchy[idx][0] )
					{
						cv::Scalar color( rand()&255, rand()&255, rand()&255 );
						cv::drawContours( dst, contours, idx, color, 5, 8, hierarchy );
					}

					if (approx.size() == 3)
					{
						obj = "Triangle";
						setLabel(dst, obj, contours[k]);    // Triangles
					}
					else if (approx.size() >= 4 && approx.size() <= 6)
					{
						// Number of vertices of polygonal curve
						int vtc = approx.size();

						// Get the cosines of all corners
						std::vector<double> cos;
						for (int j = 2; j < vtc+1; j++)
							cos.push_back(angle(approx[j%vtc], approx[j-2], approx[j-1]));

						// Sort ascending the cosine values
						std::sort(cos.begin(), cos.end());

						// Get the lowest and the highest cosine
						double mincos = cos.front();
						double maxcos = cos.back();

						// Use the degrees obtained above and the number of vertices
						// to determine the shape of the contour
						if ((vtc >= 4 && vtc <= 6) && mincos >= -0.1 && maxcos <= 0.3)
						{
							obj = "Cube";
							setLabel(dst, obj, contours[k]);
						}
						else if (vtc == 6 && mincos >= -0.93 && maxcos <= -0.34)
						{
							obj = "Cube";
							setLabel(dst, obj, contours[k]);
						}
						else if (vtc == 6 && mincos >= -0.55 && maxcos <= -0.45)
						{
							obj = "Cube";
							setLabel(dst, obj, contours[k]);
						}
					}
					else
					{
						// Detect and label circles
						double area = cv::contourArea(contours[k]);
						cv::Rect r = cv::boundingRect(contours[k]);
						int radius = r.width / 2;

						if (std::abs(1 - ((double)r.width / r.height)) <= 0.4 &&
						std::abs(1 - (area / (CV_PI * std::pow(radius, 2)))) <= 0.4){
							obj = "Ball";
							setLabel(dst, obj, contours[k]);
						}
					}
				}

				//Calculate the moments of the thresholded image
				oMoments[i] = moments(imgThresholded[i]);

				dM01[i] = oMoments[i].m01;
				dM10[i] = oMoments[i].m10;
				dArea[i] = oMoments[i].m00;

				if (dArea[i] > 20000 && obj.compare("")!=0)
				{
					//calculate the position of the object
					posX[i] = dM10[i] / dArea[i];
					posY[i] = dM01[i] / dArea[i];


					if (posX[i] >= 0 && posY[i] >= 0)
					{
						// Draw an example circle on the video stream
						//					if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
						//					{
						//						cv::circle(cv_ptr->image, cv::Point(posX[i], posY[i]), 10, CV_RGB(255,0,0));
						//						ROS_INFO("Color detected = %s \n", color[i].c_str());
						if((color[i].compare("Green")==0 && obj.compare("Ball")==0) || (color[i].compare("Light Green")==0 && obj.compare("Ball")==0))
							obj = "Cylinder";
						else if(color[i].compare("Blue")==0 && obj.compare("Triangle")==0)
							obj = "Triangle";
						//						else if(color[i].compare("Yellow")==0 && (obj.compare("Triangle")==0 || obj.compare("Patric")==0 || obj.compare("Cylinder")==0 || obj.compare("Cross")==0))
						//							obj = "Unknown";
						else if(color[i].compare("Orange")==0)
							obj = shape[4];
						else if(color[i].compare("Purple")==0)
							obj = shape[5];

						ROS_INFO("Object detected = %s %s \n", color[i].c_str(), obj.c_str());

						//					}
					}
				}
			}
		}
		// Update GUI Window
		cv::imshow(OPENCV_WINDOW, cv_ptr->image);
		//cv::imshow("HSV",hsv);
		cv::imshow("dst",dst);
		cv::imshow("Image Thresholded",imgThresholded[0]);
		cv::waitKey(3);

		// Output modified video stream
		image_pub_.publish(cv_ptr->toImageMsg());
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_converter");
	objshape objshape_node;
	ros::Rate loop_rate(CTRL_FREQ*2);

	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

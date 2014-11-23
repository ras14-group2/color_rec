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
#include <ocv_msgs/ocv.h>



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
	ros::Publisher ocv_pub_;
	ros::Publisher sound_pub_;

private:

public:

	cv::Mat hsv, imgThresholded[6],imgThresholded_new[6], dst;
	int iLowH;
	int iHighH;

	int iLowS;
	int iHighS;

	int iLowV;
	int iHighV;

	cv::Moments oMoments[6];

	double dM01[6];
	double dM10[6];
	double dArea[6];

	int posX[6];
	int posY[6];

	int count;

	std::string obj, preobj;
	std_msgs::String obj_msgs;
	std_msgs::String msgrec;
	ocv_msgs::ocv ocvmgs;

	objshape()
	: it_(n_), iLowH(0), iHighH(179), iLowS(0), iHighS(255), iLowV(0), iHighV(255), count(0)
	{
		// Subscribe to input video feed and publish output video feed
		image_sub_ = it_.subscribe("camera/rgb/image_raw", 1, &objshape::imageCb, this); // /camera/image_raw
		image_pub_ = it_.advertise("/image_converter/output_video", 1);
		ocv_pub_= n_.advertise<ocv_msgs::ocv>("/ocvrec/strings", 1);
		sound_pub_= n_.advertise<std_msgs::String>("/espeak/string", 1);

		//	cv::namedWindow(OPENCV_WINDOW);
	}

	~objshape()
	{
		//	cv::destroyWindow(OPENCV_WINDOW);
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		static const std::string color[] = {"Yellow", "Orange", "Green", "Blue", "Purple", "Red"}; // Colors;
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

		cv::medianBlur(hsv,hsv,5);

		inRange(hsv, cv::Scalar(11, 107, 212), cv::Scalar(29, 255, 255), imgThresholded_new[0]); //Threshold the image Yellow
		//		inRange(hsv, cv::Scalar(11, 150, 150), cv::Scalar(29, 255, 255), imgThresholded_new[0]); //Threshold the image Yellow
		//		inRange(hsv, cv::Scalar(13, 164, 164), cv::Scalar(29, 255, 255), imgThresholded_new[0]); //Threshold the image Yellow
		inRange(hsv, cv::Scalar(0, 170, 170), cv::Scalar(10, 255, 255), imgThresholded_new[1]); //Threshold the image Orange
		inRange(hsv, cv::Scalar(30, 80, 60), cv::Scalar(89, 255, 255), imgThresholded_new[2]); //Threshold the image Green
		inRange(hsv, cv::Scalar(90, 29, 60), cv::Scalar(119, 255, 255), imgThresholded_new[3]); //Threshold the image Blue
		inRange(hsv, cv::Scalar(120, 90, 80), cv::Scalar(160, 255, 255), imgThresholded_new[4]); //Threshold the image Purple
		inRange(hsv, cv::Scalar(161, 50, 50), cv::Scalar(179, 255, 240), imgThresholded_new[5]); //Threshold the image Red

		if(count < 6)
		{
			for(int i=0; i<6 ; i++)
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

			for(int i=0; i<6 ; i++){

				// Find contours
				std::vector<std::vector<cv::Point> > contours;
				cv::findContours(imgThresholded[i].clone() , contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

				std::vector<cv::Point> approx;


				if(color[i].compare("Orange") != 0 && color[i].compare("Purple") != 0)
				{

					for (int k = 0; k < contours.size(); k++)
					{
						obj = "";
						// Approximate contour with accuracy proportional
						// to the contour perimeter
						cv::approxPolyDP(cv::Mat(contours[k]), approx, cv::arcLength(cv::Mat(contours[k]), true)*0.025, true); //0.02

						// Skip small or non-convex objects
						if (std::fabs(cv::contourArea(contours[k])) < 100 || std::fabs(cv::contourArea(contours[k])) > 10000|| approx.size() < 3 || !cv::isContourConvex(approx))
						{                                            // 1000                                            //10000
							continue;
						}

						for(size_t j = 0; j < approx.size() - 1; j++){
							cv::line(dst, approx[j], approx[j+1], cv::Scalar(0, 0, 0), 3);
						}

						cv::line(dst, approx[approx.size()-1], approx[0], cv::Scalar(0, 0, 0), 3);

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

						if (approx.size() == 3 || (approx.size() == 5 && mincos > -0.8))
						{
							obj = "Triangle";
							setLabel(dst, obj, contours[k]);    // Triangles
						}
						else if (approx.size() >= 4 && approx.size() <= 6)
						{
							// Use the degrees obtained above and the number of vertices
							// to determine the shape of the contour
							if (mincos >= -0.93 && maxcos <= 0.3)
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

							if (std::abs(1 - ((double)r.width / r.height)) <= 0.6 &&
							std::abs(1 - (area / (CV_PI * std::pow(radius, 2)))) <= 0.6)
							{
								obj = "Ball";
								setLabel(dst, obj, contours[k]);
							}
						}
						//Calculate the moments of the thresholded image
						oMoments[i] = moments(imgThresholded[i]);

						dM01[i] = oMoments[i].m01;
						dM10[i] = oMoments[i].m10;
						dArea[i] = oMoments[i].m00;

						if (dArea[i] > 10000 && obj.compare("")!=0)
						{
							//calculate the position of the object
							posX[i] = dM10[i] / dArea[i];
							posY[i] = dM01[i] / dArea[i];

							if (posX[i] >= 0 && posY[i] >= 0)
							{
								if(color[i].compare("Green")==0 && obj.compare("Ball")==0)
									obj = "Cylinder";
								//								else if(color[i].compare("Blue")==0 && obj.compare("Triangle")==0)
								//									obj = "Triangle";
								else if(obj.compare("Triangle")==0 && color[i].compare("Blue") != 0)
								{
									obj = "Cube";
								}
								if(preobj.compare(obj.c_str())!=0){
									ROS_INFO("Object detected = %s \n", (color[i]+' '+obj).c_str());
									preobj = obj;
									//publish messages color and shape
									msgrec.data = color[i].c_str();
									ocvmgs.color = msgrec;
									msgrec.data = obj.c_str();
									ocvmgs.shape = msgrec;
									ocv_pub_.publish(ocvmgs);
									//publish sound message
							    obj_msgs.data=color[i]+' '+obj;
						      sound_pub_.publish(obj_msgs);
								}
							}
						}
					}//end of loop over all contours
				}
				else //-->(color[i].compare("Orange") == 0 || color[i].compare("Purple") == 0)
				{
					//Calculate the moments of the thresholded image
					oMoments[i] = moments(imgThresholded[i]);

					//dM01[i] = oMoments[i].m01;
					//dM10[i] = oMoments[i].m10;
					dArea[i] = oMoments[i].m00;

					if (dArea[i] > 10000)
					{

						if(color[i].compare("Orange")==0)
						{
							obj = "Patric";
						}
						else{
							obj = "Cross";
						}
						if(preobj.compare(obj.c_str())!=0){
							ROS_INFO("Object detected = %s \n", (color[i]+' '+obj).c_str());
							ROS_INFO("YEI AN OBJ");
							preobj = obj;
							//publish messages color and shape
							msgrec.data = color[i].c_str();
							ocvmgs.color = msgrec;
							msgrec.data = obj.c_str();
							ocvmgs.shape = msgrec;
							ocv_pub_.publish(ocvmgs);
							//publish sound message
							obj_msgs.data=color[i]+' '+obj;
						  sound_pub_.publish(obj_msgs);
						}
					}
				}//end of distinguish between orange/purple and other colors
			}//end of loop over all colors
		} //end of if(count==6)


		//	Update GUI Window
		//	cv::imshow(OPENCV_WINDOW, cv_ptr->image);
		cv::imshow("HSV",hsv);
		cv::imshow("dst",dst);

		cv::imshow("Yellow",imgThresholded[1]);
		cv::imshow("Green",imgThresholded[2]);
		cv::waitKey(3);

		// Output modified video stream
		image_pub_.publish(cv_ptr->toImageMsg());
		std_msgs::String obj_msgs;
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "objshape_node");
	objshape objshape_node;
	ros::Rate loop_rate(10*5);

	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}


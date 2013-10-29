/*
 * main.cpp
 *
 *  Created on: 4 Jun 2013
 *      Author: Miguel A. Olivares-Mendez
 *      SnT - University of Luxembourg
 *      Automation Research Group
 *
 *		This is a version of ArUco software for ROS
 *		The ArUco software is a C++ library developed by Rafael Muñoz Salinas (2011).
 *		http://sourceforge.net/projects/aruco/files/1.2.4/
 *		This library let you create augmented reality codes (and board codes) and detect
 *		them using a monocular camera.
 *
 *		An application of this AR code detection for control an UAV for See and Avoid task
 *		has done by Miguel A. Olivares-Mendez
 */

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Image.h"

//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>

//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
//Include headers for OpenCV Image processing
#include <opencv2/imgproc/imgproc.hpp>
//Include headers for OpenCV GUI handling
#include <opencv2/highgui/highgui.hpp>
#include "aruco/aruco.h"
#include "aruco/cvdrawingutils.h"
#include "aruco/marker_info.h"
//#include "aruco/cameraparameters.h"
//#include "aruco/aruco_gl.h"
#include <cstdlib>
#include <string.h>

//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;

//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "Image Processed";

//Use method of ImageTransport to create image publisher
image_transport::Publisher 	pub;
ros::Publisher 				pubOutput;
ros::ServiceClient 			client;
std_msgs::Float32 			output;
//Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
cv_bridge::CvImagePtr 		cv_ptr;
//cv::Mat resultGray, result;
IplImage 					img;
cv::Mat 					imgmat, TheInputImageCopy;
cv::Point 					pt1;
double 						targetLocation[3];
//ros::Publisher imagePorcessedPub;
// Global variables (modified by topic subscribers):
bool 						simulationRunning=true;
bool 						sensorTrigger=false;
float 						simulationTime=0.0f;
char 						*serviceName, *cameraSubscriber, *controlPublisher;
//aruco
vector<aruco::Marker> 		TheMarkers;
aruco::MarkerDetector 		MDetector;
aruco::CameraParameters 	TheCameraParameters;
float			 			TheMarkerSize;
int 						ThePyrDownLevel;
unsigned int 				jj;
cv::Point2f 				center;
double 						area;
aruco::marker_info			msg_marker;
float 						focalX, focalY;

const char *location = "/home/miguelolivaresmendez/code/camerasCalibration/";
char calibration[100];
char *cameraFile;
ros::Publisher marker_pub;


//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
	try	{
		//Always copy, returning a mutable CvImage
		//OpenCV expects color images to use BGR channel order.
		cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
	}	catch (cv_bridge::Exception& e)	{
		//if there is an error during conversion, display it
		ROS_ERROR("ueyecamera_imgProcessing::main.cpp::cv_bridge exception: %s", e.what());
		return;
	}
    double thres1 = 12.0; //10.0
    double thres2 = 3.0;	//8.0
	while (!msg_marker.id.empty()){
	  msg_marker.id.pop_back();
	  msg_marker.centerX.pop_back();
	  msg_marker.centerY.pop_back();
	  msg_marker.area.pop_back();
	  msg_marker.perimeter.pop_back();
	  msg_marker.rotX.pop_back();
	  msg_marker.rotY.pop_back();
	  msg_marker.rotZ.pop_back();
	  msg_marker.distance.pop_back();
	  msg_marker.trsX.pop_back();
	  msg_marker.trsY.pop_back();
	}
	msg_marker.numberOfMarkers = 0;


	img = cv_ptr->image;
	CvSize imgSize = cvGetSize(&img);
	imgmat = cv::Mat(&img, true);
	TheCameraParameters.resize(imgmat.size());
//Configure other parameters
	if (ThePyrDownLevel>0)
		MDetector.pyrDown(ThePyrDownLevel);

	MDetector.getThresholdParams( thres1, thres2);
	MDetector.detect(imgmat,TheMarkers,TheCameraParameters,TheMarkerSize);
//print marker info and draw the markers in image
	imgmat.copyTo(TheInputImageCopy);
	for (jj=0;jj<TheMarkers.size();jj++) {
	  center = TheMarkers[jj].getCenter();
	  area = TheMarkers[jj].getArea();
	  if (  TheCameraParameters.isValid()){
		  aruco::CvDrawingUtils::draw3dCube(TheInputImageCopy,TheMarkers[jj],TheCameraParameters);
		  aruco::CvDrawingUtils::draw3dAxis(TheInputImageCopy,TheMarkers[jj],TheCameraParameters);
	  }else{
			 TheMarkers[jj].draw(TheInputImageCopy,cv::Scalar(0,0,255),2);
	  }
	  msg_marker.id.push_back(TheMarkers[jj].getId());
	  msg_marker.centerX.push_back(center.x);
	  msg_marker.centerY.push_back(center.y);
	  msg_marker.area.push_back(area);
	  msg_marker.perimeter.push_back(TheMarkers[jj].getPerimeter());
	  msg_marker.rotX.push_back(TheMarkers[jj].getXrot());
	  msg_marker.rotY.push_back(TheMarkers[jj].getYrot());
	  msg_marker.rotZ.push_back(TheMarkers[jj].getZrot());
	  msg_marker.distance.push_back(TheMarkers[jj].getTZ());
	  msg_marker.trsX.push_back(TheMarkers[jj].getTX());
	  msg_marker.trsY.push_back(TheMarkers[jj].getTY());


	}
	msg_marker.numberOfMarkers = jj;

	printf("\nindex: %d,Area: %f, Center: x=%f, y=%f\n",jj,area,center.x, center.y);
//draw a 3d cube in each marker if there is 3d info
/*	if (  TheCameraParameters.isValid())
	  for (unsigned int ii=0;ii<TheMarkers.size();ii++) {
		  printf("printing cube\n");
		  aruco::CvDrawingUtils::draw3dCube(TheInputImageCopy,TheMarkers[ii],TheCameraParameters);
		  aruco::CvDrawingUtils::draw3dAxis(TheInputImageCopy,TheMarkers[ii],TheCameraParameters);
	  }*/

	targetLocation[0]=center.x;
	targetLocation[1]=center.y;


	//Display the image using OpenCV
	//cv::imshow("in",TheInputImageCopy);

	//cv::imshow(WINDOW, cv_ptr->image);
	cv::imshow(WINDOW, TheInputImageCopy);
	//Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
	cv::waitKey(1);
	/**
	* The publish() function is how you send messages. The parameter
	* is the message object. The type of this object must agree with the type
	* given as a template parameter to the advertise<>() call, as was done
	* in the constructor in main().
	*/
	//Convert the CvImage to a ROS image message and publish it on the "camera/image_processed" topic.
    pub.publish(cv_ptr->toImageMsg());
    marker_pub.publish(msg_marker);
}


int main(int argc, char **argv) {

	ros::init(argc, argv, "aruco");
	strcpy(calibration,location);
	if (argc >= 3){
	  cameraSubscriber = argv[1];
	  TheMarkerSize = atof(argv[2]);
	  cameraFile = argv[3];
	  strcat(calibration,cameraFile);
	}else{
		ROS_INFO("aruco need three parameters: camera_subscriber Aruco_marker_Size calibration_File");
		return 0;
	}
	TheCameraParameters.readFromXMLFile(calibration);
	focalX = TheCameraParameters.getCamerafx();
	focalY = TheCameraParameters.getCamerafy();

	ros::NodeHandle nh;
	//Create an ImageTransport instance, initializing it with our NodeHandle.
	image_transport::ImageTransport it(nh);
	pub = it.advertise("camera/image_processed", 1);

	ros::NodeHandle n;
	marker_pub = n.advertise<aruco::marker_info>("marker_info", 1000);

	//OpenCV HighGUI call to create a display window on start-up.
	cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
	cvNamedWindow("iplImage_window", CV_WINDOW_AUTOSIZE);

	//  image_transport::Subscriber sub = it.subscribe("vrep/QRfrontCameraData", 1, imageCallback);
	image_transport::Subscriber sub = it.subscribe(cameraSubscriber, 10, imageCallback);
	cv::destroyWindow(WINDOW);
	cvDestroyWindow("iplImage_window");

	ros::NodeHandle node("~");
	printf("Image processing of the ueye camera just started with node name ueyecamera_imgProcessing \n");

//	while (ros::ok()){
//		marker_pub.publish(msg_marker);
//
//	//	msg_marker.numberOfMarkers = 0;
//		ros::spinOnce();
//	}
	ros::spin();
  return 0;
}




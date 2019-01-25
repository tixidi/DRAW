
/*

1、kcf单点的程序
*/
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/RegionOfInterest.h>
#include "geometry_msgs/Twist.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <ros/ros.h>
#include <pcl/visualization/cloud_viewer.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>

#include <pcl/filters/voxel_grid.h>
#include <Eigen/Geometry>
#include <boost/format.hpp>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/Transform.h>

#include <stdio.h>




#define CV_PI   3.1415926535897932384626433832795
#define CV_2PI  6.283185307179586476925286766559
using namespace std;

//pcl::PointCloud<pcl::PointXYZRGB> cloud_old;
pcl::PointCloud<pcl::PointXYZRGB> cloud_aligned;
static const std::string RGB_WINDOW = "RGB Image window";

int flag=0;

int num;
int roi_x;
int roi_y;
int roi_height;
int roi_width;

cv::Mat rgbimage;
cv::Mat depthimage;
cv::Rect selectRect;
cv::Point origin;
cv::Point pointInterest;

cv::Rect result;
bool select_flag = false;
bool bRenewROI = false;  // the flag to enable the implementation of KCF algorithm for the new chosen ROI
bool bBeginKCF = false;
bool enable_get_depth = false;


using namespace std;
using namespace cv;


void onMouse(int event, int x, int y, int, void*)
{
    // if (select_flag)
    // {
    //     selectRect.x = MIN(origin.x, x);        
    //     selectRect.y = MIN(origin.y, y);
    //     selectRect.width = abs(x - origin.x);   
    //     selectRect.height = abs(y - origin.y);
    //     selectRect &= cv::Rect(0, 0, rgbimage.cols, rgbimage.rows);
    // }
    if (event == CV_EVENT_LBUTTONDOWN)
    {

        pointInterest= Point(x,y);  
        circle(rgbimage,pointInterest,2,Scalar(255,0,0,0),CV_FILLED,CV_AA,0);//划圆  
        imshow(RGB_WINDOW,rgbimage);  

    }
    else if (event == CV_EVENT_LBUTTONUP)
    {
        // select_flag = false;
        // bRenewROI = true;


        // waitKey(0);  
    }
}

class cloudHandler
{
public:
    cloudHandler()
    : viewer("Cloud Viewer"),it_(nh)
    {

        pcl_sub = nh.subscribe("/camera/depth_registered/points", 100, &cloudHandler::cloudCB, this);
        viewer_timer = nh.createTimer(ros::Duration(3), &cloudHandler::timerCB, this);

        roi_sub = nh.subscribe("/kcf/roi", 1, &cloudHandler::roiCb, this);
        targetToBaseTrans_ = nh.advertise<geometry_msgs::Transform>("/ur_driver/effort_pose",1000);
        // Subscrive to input video feed and publish output video feed

       image_sub_ = it_.subscribe("/camera/color/image_raw", 100,  &cloudHandler::imageCb, this);


        cv::namedWindow(RGB_WINDOW);

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

        cv_ptr->image.copyTo(rgbimage);

        cv::setMouseCallback(RGB_WINDOW, onMouse, (void* )0);


        cv::imshow(RGB_WINDOW, rgbimage);
        cv::waitKey(2);
  }


    void cloudCB(const sensor_msgs::PointCloud2 &input)
    {
		pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::fromROSMsg(input, cloud);

 		
		if(flag==0){
        	viewer.showCloud(cloud.makeShared());

		}
		// float center_x=roi_x+roi_width/2; //roi中心 x
		// float center_y=roi_y+roi_height/2;//roi中心 y

		float center_x=pointInterest.x; //roi中心 x
		float center_y=pointInterest.y;//roi中心 y
        
        
        cout<<"the cloud width is x  "<<cloud.width<<endl;
        cout<<"the cloud width is y  "<<cloud.height<<endl;
        cout<<"the cloud width is z  "<<cloud.points.size()<<endl;

//----------------------------------------------------------------------------------

        float from_pixel_center[2]= { center_x , center_y };


        int from_pixel_center_index = from_pixel_center[0] *640 +from_pixel_center[1];




        Eigen::Vector3d center3D,axisEndX,axisEndY;


        center3D[0]=(float)cloud.points[from_pixel_center_index].x;
        center3D[1]=(float)cloud.points[from_pixel_center_index].y;
        center3D[2]=(float)cloud.points[from_pixel_center_index].z;


        std::cerr <<"from_pixel_center " <<from_pixel_center[0]<<" "<<from_pixel_center[1]<<std::endl;

        std::cerr <<"center is        " << center3D[0] <<" "  <<center3D[1]<<" " <<center3D[2]<<std::endl;








 //-------------------------------------------------------------------------------------

        tf::StampedTransform transform;
        transform.setIdentity();
        transform.child_frame_id_ ="/vein_point";
        transform.frame_id_ = "/camera_color_optical_frame";
        transform.stamp_ = ros::Time::now();
        transform.setOrigin(tf::Vector3(center3D[0], center3D[1],center3D[2]));
        tf::Quaternion q;
        q.setRPY(0.0,0.0,0.0);//没有旋转
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform));



        // // transforms.push_back(transform);
        tf::StampedTransform pose;
        tf::StampedTransform poseCam;
        tf::StampedTransform poseTaget;
        geometry_msgs::Transform targetToBasePose;

        try
        {
                // Get transformation from "object_#" frame to target frame "map"
                // The timestamp matches the one sent over TF
                tfListener_.lookupTransform("/camera_link", "/vein_point", ros::Time(0), pose);
                tfListener_.lookupTransform("/camera_color_optical_frame", "/vein_point", ros::Time(0), poseCam);
                tfListener_.lookupTransform("/base_link", "/vein_point", ros::Time(0), poseTaget);

        }
        catch(tf::TransformException & ex)
        {
                ROS_WARN("%s",ex.what());
                // continue;
        }

        // Here "pose" is the position of the object "id" in "/map" frame.
        ROS_INFO("%s [x,y,z] [x,y,z,w] in \"%s\" frame: [%f,%f,%f] [%f,%f,%f,%f]",
                        "/camera_link", "/vein_point",
                        pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z(),
                        pose.getRotation().x(), pose.getRotation().y(), pose.getRotation().z(), pose.getRotation().w());
        ROS_INFO("%s [x,y,z] [x,y,z,w] in \"%s\" frame: [%f,%f,%f] [%f,%f,%f,%f]",
                        "/camera_color_optical_frame","/vein_point",
                        poseCam.getOrigin().x(), poseCam.getOrigin().y(), poseCam.getOrigin().z(),
                        poseCam.getRotation().x(), poseCam.getRotation().y(), poseCam.getRotation().z(), poseCam.getRotation().w());

        ROS_INFO("%s [x,y,z] [x,y,z,w] in \"%s\" frame: [%f,%f,%f] [%f,%f,%f,%f]",
                        "/base_link","/vein_point",
                        poseTaget.getOrigin().x(), poseTaget.getOrigin().y(), poseTaget.getOrigin().z(),
                        poseTaget.getRotation().x(), poseTaget.getRotation().y(), poseTaget.getRotation().z(), poseTaget.getRotation().w());


        targetToBasePose.translation.x=poseTaget.getOrigin().x();
        targetToBasePose.translation.y=poseTaget.getOrigin().y();
        targetToBasePose.translation.z=poseTaget.getOrigin().z();

        targetToBasePose.rotation.x=poseTaget.getRotation().x();
        targetToBasePose.rotation.y=poseTaget.getRotation().y();
        targetToBasePose.rotation.z=poseTaget.getRotation().z();
        targetToBasePose.rotation.w=poseTaget.getRotation().w();



        targetToBaseTrans_.publish(targetToBasePose);
 //-------------------------------------------------------------------------------------



	
    }
	void roiCb(const sensor_msgs::RegionOfInterest& msg) 
	{
		roi_x=msg.x_offset;
		roi_y=msg.y_offset;
		roi_height=msg.height;
		roi_width=msg.width;

		std::cerr<<"the roi position is " <<"roi X " <<roi_x <<"roi Y " << roi_y<<" roi_height " <<roi_height << "roi_width " <<roi_width<<std::endl;  
	
		
	}

    void timerCB(const ros::TimerEvent&)
    {
        if (viewer.wasStopped())
        {
            ros::shutdown();
        }
    }

protected:
    ros::NodeHandle nh;
    image_transport::ImageTransport it_;
    ros::Subscriber pcl_sub;
    pcl::visualization::CloudViewer viewer;
    //pcl::visualization::CloudViewer viewer_old;
    ros::Timer viewer_timer;
	ros::Subscriber roi_sub;
    tf::TransformBroadcaster br;
    ros::Publisher targetToBaseTrans_;
    tf::TransformListener tfListener_;
    image_transport::Subscriber image_sub_;

        
};

main (int argc, char **argv)
{
    ros::init (argc, argv, "pcl_visualize");

    cloudHandler handler;

    ros::spin();

    return 0;
}

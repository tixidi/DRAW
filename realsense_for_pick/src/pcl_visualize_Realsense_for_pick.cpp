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


#define CV_PI   3.1415926535897932384626433832795
#define CV_2PI  6.283185307179586476925286766559
using namespace std;

//pcl::PointCloud<pcl::PointXYZRGB> cloud_old;
pcl::PointCloud<pcl::PointXYZRGB> cloud_aligned;


int flag=0;

int num;
int roi_x;
int roi_y;
int roi_height;
int roi_width;


class cloudHandler
{
public:
    cloudHandler()
    : viewer("Cloud Viewer")
    {
        pcl_sub = nh.subscribe("/camera/depth_registered/points", 100, &cloudHandler::cloudCB, this);
        viewer_timer = nh.createTimer(ros::Duration(3), &cloudHandler::timerCB, this);

        roi_sub = nh.subscribe("/kcf/roi", 1, &cloudHandler::roiCb, this);
        targetToBaseTrans_ = nh.advertise<geometry_msgs::Transform>("/ur_driver/effort_pose",1000);

    }

    void cloudCB(const sensor_msgs::PointCloud2 &input)
    {
		pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::fromROSMsg(input, cloud);

 		
		if(flag==0){
        	viewer.showCloud(cloud.makeShared());

		}
		float center_x=roi_x+roi_width/2; //roi中心 x
		float center_y=roi_y+roi_height/2;//roi中心 y


        cout<<"the cloud width is x  "<<cloud.width<<endl;
        cout<<"the cloud width is y  "<<cloud.height<<endl;
        cout<<"the cloud width is z  "<<cloud.points.size()<<endl;

//----------------------------------------------------------------------------------

        float from_pixel_center[2]= { center_x , center_y };
        float from_pixel_xAxis[2] = { center_x+roi_width*1/4, center_y };
        float from_pixel_yAxis[2] = { center_x, center_y+roi_height*1/4};

        int from_pixel_center_index = from_pixel_center[0] *640 +from_pixel_center[1];
        int from_pixel_xAxis_index  = from_pixel_xAxis[0] * 640+from_pixel_xAxis[1];
        int from_pixel_yAxis_index  = from_pixel_yAxis[0] *640+from_pixel_yAxis[1];



        Eigen::Vector3d center3D,axisEndX,axisEndY;


        center3D[0]=(float)cloud.points[from_pixel_center_index].x;
        center3D[1]=(float)cloud.points[from_pixel_center_index].y;
        center3D[2]=(float)cloud.points[from_pixel_center_index].z;

        axisEndX[0]=(float)cloud.points[from_pixel_xAxis_index].x;
        axisEndX[1]=(float)cloud.points[from_pixel_xAxis_index].y;
        axisEndX[2]=(float)cloud.points[from_pixel_xAxis_index].z;

        axisEndY[0]=(float)cloud.points[from_pixel_yAxis_index].x;
        axisEndY[1]=(float)cloud.points[from_pixel_yAxis_index].y;
        axisEndY[2]=(float)cloud.points[from_pixel_yAxis_index].z;

        std::cerr <<"from_pixel_center " <<from_pixel_center[0]<<" "<<from_pixel_center[1]<<std::endl;
        std::cerr <<"from_pixel_xAxis  " <<from_pixel_xAxis[0]<<" "<<from_pixel_xAxis[1]<<std::endl;
        std::cerr <<"from_pixel_yAxis  " <<from_pixel_yAxis[0]<<" "<<from_pixel_yAxis[1]<<std::endl;

        std::cerr <<"center is        " << center3D[0] <<" "  <<center3D[1]<<" " <<center3D[2]<<std::endl;
        std::cerr <<"axisEndX is      " << axisEndX[0] <<" "  <<axisEndX[1]<<" " <<axisEndX[2]<<std::endl;
        std::cerr <<"axisEndY is      " << axisEndY[0] <<" "  <<axisEndY[1]<<" " <<axisEndY[2]<<std::endl;







 //-------------------------------------------------------------------------------------

        tf::StampedTransform transform;
        transform.setIdentity();
        transform.child_frame_id_ ="/vein_point";
        transform.frame_id_ = "/camera_color_optical_frame";
        transform.stamp_ = ros::Time::now();
        transform.setOrigin(tf::Vector3(center3D[0], center3D[1],center3D[2]));

        //set rotation
        tf::Vector3 xAxis(axisEndX[0] - center3D[0], axisEndX[1] - center3D[1], axisEndX[2] - center3D[2]);
        xAxis.normalize();
        tf::Vector3 yAxis(axisEndY[0] - center3D[0], axisEndY[1] - center3D[1], axisEndY[2] - center3D[2]);
        yAxis.normalize();
        tf::Vector3 zAxis = xAxis.cross(yAxis);
        zAxis.normalize();
        tf::Matrix3x3 rotationMatrix(
                                xAxis.x(), yAxis.x() ,zAxis.x(),
                                xAxis.y(), yAxis.y(), zAxis.y(),
                                xAxis.z(), yAxis.z(), zAxis.z());
        tf::Quaternion q;
        rotationMatrix.getRotation(q);
        // set x axis going front of the object, with z up and z left
        q *= tf::createQuaternionFromRPY(CV_PI/2.0, CV_PI/2.0, 0);
        transform.setRotation(q.normalized());
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
    ros::Subscriber pcl_sub;
    pcl::visualization::CloudViewer viewer;
    //pcl::visualization::CloudViewer viewer_old;
    ros::Timer viewer_timer;
	ros::Subscriber roi_sub;
    tf::TransformBroadcaster br;
    ros::Publisher targetToBaseTrans_;
    tf::TransformListener tfListener_;

        
};

main (int argc, char **argv)
{
    ros::init (argc, argv, "pcl_visualize");

    cloudHandler handler;

    ros::spin();

    return 0;
}

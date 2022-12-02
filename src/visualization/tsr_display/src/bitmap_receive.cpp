/*
 * Copyright (c) 2017, FEV Polska. All rights reserved.
 *
 * ROS node for displaying detected roadsigns in rviz
 * log:
 * 05-09-2017: initial version, moskal@fev.com
*/

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include "vision_msgs/od_data.h"
#include <string>

constexpr char kFrameID[] = "/car_cam";

vision_msgs::od_data imgs;
bool new_msg=false;

void image_callback(const vision_msgs::od_data::ConstPtr &msg)
{
    imgs = *msg;
    new_msg=true;
}

std::vector<cv::Mat> load_icons(std::string ros_path)
{
    std::vector<cv::Mat> icons;

    std::vector<std::string> types;
    types.push_back("sign");
    types.push_back("car");
    types.push_back("pedestrian");
    types.push_back("bicycle");

    for(int i=0;i<types.size();i++)
    {
        std::string file_name=ros_path+"/cfg/icons/"+types[i]+".png";
        cv::Mat temp=cv::imread(file_name,-1);
        if((temp.cols!=0)&&(temp.rows!=0)) resize(temp, temp, cv::Size(32,32));
        icons.push_back(temp);
        ROS_INFO("icon %s loaded %dx%d %d",file_name.c_str(),temp.rows,temp.cols,temp.type());
    }

    return icons;
}

int main(int argc, char** argv)
{
    std::string ros_path;

    ros::init(argc, argv, "image3d_publisher");
    ros::NodeHandle n;

    n.param("/tsr_display/workspace_path", ros_path, std::string("./"));
    std::vector<cv::Mat> icons=load_icons(ros_path);

    ros::Subscriber sub = n.subscribe("/adas_node/od_data", 1, image_callback);

    ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("/tsr_data", 1);

    ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("/od_markers", 10);

    ros::NodeHandle param_handl(n, "roadsigns");
    std::string frame_id = "";
    param_handl.param<std::string>("frame_id", frame_id, std::string(kFrameID));

    ros::Rate loop_rate(60);

    while (ros::ok())
    {
        if(new_msg)
        {
            sensor_msgs::PointCloud2 pc;

            int npts = 0;

            // calculate the total number of points (pixels) in images
            for (int p = 0; p < imgs.objects.size(); ++p)
            {
                if (! imgs.objects[p].valid)
                {
                    npts += 32 * 32;
                }
            }

            pc.header.frame_id = frame_id;
            pc.height = 1; // unstructured pointcloud
            pc.width = npts; // pointlcloud width is just the number of points in it
            pc.is_bigendian = false;
            pc.is_dense = false; // there may be invalid points

            // create a pointcloud modifier, set the parameter fields
            sensor_msgs::PointCloud2Modifier pcmod(pc);

            pcmod.setPointCloud2FieldsByString(2,  "xyz","rgba");

            // create iterators for each field
            sensor_msgs::PointCloud2Iterator<float> iter_x(pc, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(pc, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(pc, "z");

            sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgba(pc, "rgba");
            
            visualization_msgs::MarkerArray markerArray;

            if (imgs.camera_frame_id != 0)
            {
                

                // iterate thru all images in the message, add their points to the pointcloud
                for (int p = 0; p < imgs.objects.size(); ++p)
                {
                    if (! imgs.objects[p].valid)
                    {
                        visualization_msgs::Marker cube;
                        cube.header.frame_id = "/car_cam";
                        cube.header.stamp = ros::Time::now();
                        cube.ns = "/car_view";
                        cube.action = visualization_msgs::Marker::DELETE;

                        cube.id = p;

                        cube.pose.orientation.w = 1.0;

                        cube.type = visualization_msgs::Marker::CUBE;

                        markerArray.markers.push_back(cube);
                        continue;
                    }
                    int type=imgs.objects[p].type;

                    // store image data in local variables
                    uint32_t vpts = 32;
                    uint32_t hpts = 32;

                    float blcy = imgs.objects[p].real_y;
                    float blcz = imgs.objects[p].real_z;
                    float wdth = imgs.objects[p].height;
                    float hght = imgs.objects[p].width;
                    float dist = imgs.objects[p].real_x;

                    float vstep = hght/vpts;
                    float hstep = wdth/hpts;

                    float z = blcz + hght - .5f*vstep;

                    //marker array
                    visualization_msgs::Marker cube;
                    cube.header.frame_id = "/car_cam";
                    cube.header.stamp = ros::Time::now();
                    cube.ns = "/car_view";
                    cube.action = visualization_msgs::Marker::ADD;

                    cube.id = p;

                    cube.pose.position.x = imgs.objects[p].real_x;
                    cube.pose.position.y = imgs.objects[p].real_y;
                    cube.pose.position.z = imgs.objects[p].real_z;
                    cube.pose.orientation.x = 0.0;
                    cube.pose.orientation.y = 0.0;
                    cube.pose.orientation.z = 0.0;
                    cube.pose.orientation.w = 1.0;
                    cube.scale.x = imgs.objects[p].height;
                    cube.scale.y = imgs.objects[p].width;
                    cube.scale.z = 1.6;
                    cube.color.a = 1.0; // Don't forget to set the alpha!
                    cube.color.r = 0.0;
                    cube.color.g = 1.0;
                    cube.color.b = 0.0;

                    if(type==0)
                    {
                        cube.color.r = 1.0;
                        cube.color.g = 0.0;
                        cube.color.b = 0.0;
                    } 
                    else if(type==1)
                    {  
                        cube.color.r = 1.0;
                        cube.color.g = 0.0;
                        cube.color.b = 1.0;
                    }
                    else if(type==2)
                    {  
                        cube.color.r = 0.0;
                        cube.color.g = 1.0;
                        cube.color.b = 1.0;
                    } 
                    else if(type==3)
                    {  
                        cube.color.r = 1.0;
                        cube.color.g = 1.0;
                        cube.color.b = 1.0;
                    }
    
/*
if((object.type==OBJ_TYPE_CAR)&&(object.sub_type==OBJ_SUBTYPE_CAR_CAR))
            {  
                color = cv::Scalar(255,0,255);
                if(visualize_cars) visualize=true;
            }
            else if((object.type==OBJ_TYPE_CAR)&&(object.sub_type==OBJ_SUBTYPE_CAR_TRUCK))
            {
                color = cv::Scalar(255,0,0);
                if(visualize_cars) visualize=true;
            }
            else if(object.type==OBJ_TYPE_PED) 
            {
                color = cv::Scalar(255,255,0);
                if(visualize_pedestrians) visualize=true;
            }
            else if((object.type==OBJ_TYPE_XCYCLE)&&(object.sub_type==OBJ_SUBTYPE_BICYCLE))
            {
                color = cv::Scalar(255,255,255);
                visualize=true;
            }
            else if((object.type==OBJ_TYPE_XCYCLE)&&(object.sub_type==OBJ_SUBTYPE_MOTORCYCLE))
            {
                color = cv::Scalar(255,255,255);
                visualize=true;
            }
            else if(object.type==OBJ_TYPE_SIGN) 
            {
                line_width=2;
                color = cv::Scalar(0,0,255);
*/

                    cube.type = visualization_msgs::Marker::CUBE;

                    markerArray.markers.push_back(cube);

                    // image data iterator
                    int k = 0;

                    for (int i = 0; i < vpts; ++i)
                    {
                        float y = blcy - .5f*hstep;

                        for (int j = 0; j < hpts; ++j)
                        {

                            // assign coordinates to a point
                           *iter_x = dist;
                            *iter_y = y;
                            *iter_z = z;
                            //printf("iter y: %.02f, z:%.02f, h:%.02f, step:%.02f\n",*iter_y,*iter_z,hght,vstep);
                            // assign color information of a point
                            
                            if((type>0)&&(type<icons.size())&&(icons[type].cols>0))
                            {
                                iter_rgba[0] = icons[type].ptr()[4*k+0];
                                iter_rgba[1] = icons[type].ptr()[4*k+1];
                                iter_rgba[2] = icons[type].ptr()[4*k+2];
                                iter_rgba[3] = icons[type].ptr()[4*k+3];
                            }
                            else
                            {
                                iter_rgba[0] = 255u;//imgs.objects[p].image.data[3*k];
                                iter_rgba[1] = 255u;//imgs.objects[p].image.data[3*k+1];
                                iter_rgba[2] = 255u;//imgs.objects[p].image.data[3*k+2];
                                iter_rgba[3] = 255u; //no alpha channel
                            }

                            // increment image data iterator
                            ++k;

                            // increment pointcloud iterators
                            ++iter_x; ++iter_y; ++iter_z; ++iter_rgba;
    
                            // move the current y coordinate
                            y -= hstep;
                        }

                        // move the current z coordinate
                        z -= vstep;
                    }

                }

            }

            // publish the pointcloud with all images
            pub.publish(pc);
            marker_pub.publish(markerArray);
            new_msg=false;
        }

        loop_rate.sleep();
        ros::spinOnce();
    }
}

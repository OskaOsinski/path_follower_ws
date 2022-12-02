#include "ros/ros.h"
#include "std_msgs/String.h"
#include <azsp_msgs/FloatPoint.h>

#include <sstream>
#include <fstream>
#include <nlohmann/json.hpp>
#include <gps_msgs/SystemState.h>

using json = nlohmann::json;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n("~");

  //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Publisher chatter_pub = n.advertise<gps_msgs::SystemState>("/gps_state", 100);

  ros::Rate loop_rate(60);

  int count = 0;
  double dx = 0;
  double dy = 0;
  double x = 0;
  double y = 0;

  //load json file
  json data;
  std::ifstream people_file("/home/mateusz/catkin_ws/src/template_gui_package-main/nodes/path/sciezka.json", std::ifstream::binary);
  people_file >> data;

  y = data["gps_ref_lat"];
  x = data["gps_ref_lon"];
  float prev_x = 0.0;
  float prev_y = 0.0;
  //std::cout << data["waypoints"].size() << std::endl;
  while (ros::ok())
  {
    gps_msgs::SystemState msg;

    if(count >= data["waypoints"].size() ){
      count = 0;
    }
    if(count == 0){
        dx = 0;
        dy = 0;
    }
    else{
      dy = data["waypoints"][count]["y"];
      dx = data["waypoints"][count]["x"];
    }
    //std::cout<<data["gps_ref_lat"] << std::endl;
    //std::cout<<data["gps_ref_lon"] << std::endl;
    
    //msg.x = 21.014118194580078 + (21.01401122-21.014585214049692)/10*count;
    //msg.y = 52.26820373535156 + (52.26802585-52.26869226538378)/10*count;

    msg.nav_sat_fix.longitude = x + dx*2.47/230000.0;
    msg.nav_sat_fix.latitude = y + dy*1.87/230000.0;
    //std::cout <<"dx " << dx << std::endl;
    //std::cout <<"roznica x " << dx-prev_x << std::endl;
    //std::cout <<"roznica y " << dy-prev_y << std::endl;
    prev_x = dx;
    prev_y = dy;
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}

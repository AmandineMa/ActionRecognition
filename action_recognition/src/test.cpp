#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <fstream>
#include "std_msgs/String.h"
void humanListCallback(const std_msgs::String::ConstPtr& msg);

void do_stuff(int* publish_rate)
{

  ros::Rate loop_rate(*publish_rate);
  while (ros::ok())
  {
    std::cout << "hey" << std::endl;
    loop_rate.sleep();
  }
}

uint64_t time_now;
int c=0;
int main(int argc, char** argv)
{
  int rate_b = 1; // 1 Hz
  uint64_t time_prev;
  
  uint64_t time_diff;

  ros::init(argc, argv, "mt_node");
  ros::NodeHandle node = ros::NodeHandle();
  ros::Publisher chatter_pub = node.advertise<std_msgs::String>("/pdg/humanList", 1);
  ros::Subscriber human_list_sub_ = node.subscribe("/pdg/humanList", 1, humanListCallback);  
  bool first = true;
  // spawn another thread
  boost::thread thread_b(do_stuff, &rate_b);
  std::ofstream file("/home/amayima/Desktop/test.txt");
  ros::Rate loop_rate(30);
  while (ros::ok())
  { 
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello world ";
    msg.data = ss.str();
    chatter_pub.publish(msg);

    ros::spinOnce();    

    time_now = ros::Time::now().toNSec();
    if(!first)
      time_diff= time_now - time_prev;
    first = false;
    time_prev = time_now;
    file << c << " " << time_diff << std::endl;
    loop_rate.sleep();   
  }

  // wait the second thread to finish
  thread_b.join();

  return 0;
}

void humanListCallback(const std_msgs::String::ConstPtr& msg){
  c++;
}

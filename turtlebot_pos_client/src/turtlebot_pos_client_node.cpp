#include <boost/asio.hpp>
#include <iostream>
#include <ros/ros.h>
#include <string>
#include <vector>
#include <stdlib.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

using namespace std;
using boost::asio::ip::udp;

//turtlebotPosServer::TagDetectionServer tag;
geometry_msgs::PoseWithCovarianceStamped tag;
/*
void copy_over_hack(const turtlebotPosServer::TagDetectionServer &tag, turtlebotPosServer::TagDetectionServer &tag2){
	tag2.header.seq = tag.header.seq;
	tag2.header.stamp = tag.header.stamp;
	tag2.tag_id = tag.tag_id;
	tag2.pose = tag.pose;
}
*/

void copy_data_to_tag_detection_server(const std::vector<char> v, int numData, int bytesPerData ){
	std::string tmp(v.begin(), v.begin()+bytesPerData);
	tag.pose.pose.position.x = atof(tmp.c_str());

	std::string tmp2(v.begin()+bytesPerData, v.begin()+(bytesPerData*2));
	tag.pose.pose.position.y = atof(tmp2.c_str());

	std::string tmp3(v.begin()+(bytesPerData*2), v.begin()+(bytesPerData*3));
	tag.pose.pose.position.z = atof(tmp3.c_str());

	std::string tmp4(v.begin()+(bytesPerData*3), v.begin()+(bytesPerData*4));
	tag.pose.pose.orientation.x = atof(tmp4.c_str());

	std::string tmp5(v.begin()+(bytesPerData*4), v.begin()+(bytesPerData*5));
	tag.pose.pose.orientation.y = atof(tmp5.c_str());

	std::string tmp6(v.begin()+(bytesPerData*5), v.begin()+(bytesPerData*6));
	tag.pose.pose.orientation.z = atof(tmp6.c_str());

	std::string tmp7(v.begin()+(bytesPerData*6), v.begin()+(bytesPerData*7));
	tag.pose.pose.orientation.w = atof(tmp7.c_str());

	cout << tmp << " " << tmp2 << " " << tmp3 << endl;
	cout << tmp4 << " " << tmp5 << " " << tmp6 << " " << tmp7 << endl;
	cout << endl;

	cout << (double)atof(tmp.c_str()) << " " << (double)atof(tmp2.c_str()) << " " << (double)atof(tmp3.c_str()) << endl;
	cout << (double)atof(tmp4.c_str()) << " " << (double)atof(tmp5.c_str()) << " " << (double)atof(tmp6.c_str()) << " " << (double)atof(tmp7.c_str()) << endl;
	cout << endl;
	cout << "-------------------------------" << endl;
}

int main(int argc, char ** argv)
{

  ros::init(argc, argv, "aprilTagServer");
  ros::NodeHandle n("~");
  //ros::Publisher tag_pub = n.advertise<turtlebotPosServer::TagDetectionServer>("/wave/tag2", 1);
  ros::Publisher tag_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/indoor_pos", 1);

  try
  {
    boost::asio::io_service io_service;
    udp::socket socket(io_service, udp::endpoint(udp::v4(), 5001));

    while (ros::ok())
    {
      udp::endpoint remote_endpoint;
      boost::system::error_code error;
      int numData = 7;
      int bytesPerData = 10;
      int totalBytes = bytesPerData * numData;
      std::vector <char> net(totalBytes,0);
      //socket.receive_from(boost::asio::buffer(&tag, sizeof(turtlebotPosServer::TagDetectionServer)),
          //remote_endpoint, 0, error);
      socket.receive_from(boost::asio::buffer((char *) &net.front(), totalBytes), remote_endpoint, 0, error);
      copy_data_to_tag_detection_server(net, numData, bytesPerData);
      //turtlebotPosServer::TagDetectionServer tag2;
 //     copy_over_hack(tag, tag2);
      tag_pub.publish(tag);
  //    cout << out << " " << out2 << endl;
    }
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }


  return 0;
}

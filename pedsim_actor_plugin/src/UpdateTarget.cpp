#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

int main(int argc, char** argv)
{
  gazebo::client::setup(argc, argv);
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  std::string topicName = "~/" + std::string(argv[1]) + "/target";
  gazebo::transport::PublisherPtr pub = node->Advertise<gazebo::msgs::Vector3d>(topicName);
  pub->WaitForConnection();

  gazebo::msgs::Vector3d msg;
  gazebo::msgs::Set(&msg, ignition::math::Vector3d(std::atof(argv[2]), std::atof(argv[3]), std::atof(argv[4])));

  pub->Publish(msg);
  gazebo::client::shutdown();
}

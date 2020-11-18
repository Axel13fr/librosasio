#include <boost/asio.hpp>
#include <librosasio/asio_callbackqueue.h>
#include <ros/ros.h>
#include "node.h"

int main(int argc, char* argv[])
{
  // SigInt is handled directly by boost asio
  boost::asio::io_service io_context;

  // Static Function Call doing the magic to unify event loops
  AsioCallbackQueue::replaceGlobalQueue(io_context);
  ros::init(argc, argv, "test_node", ros::init_options::NoSigintHandler);

  ros::NodeHandle nh("TestNode");
  TestNode node(io_context,nh);

  io_context.run();
  std::cout << "Execution finished" << std::endl;

  ros::shutdown();
  return 0;
}

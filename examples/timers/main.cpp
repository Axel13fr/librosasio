#include "node.h"
#include <boost/asio.hpp>
#include <librosasio/asio_callbackqueue.h>
#include <ros/ros.h>

int main(int argc, char* argv[])
{
    // Boost Event loop handler
#if BOOST_VERSION < 106600
    boost::asio::io_service io_context;
#else
    boost::asio::io_context io_context;
#endif

    // Static Function Call doing the magic to unify event loops
    AsioCallbackQueue::replaceGlobalQueue(io_context);
    ros::init(argc, argv, "test_node", ros::init_options::NoSigintHandler);

    ros::NodeHandle nh("TestNode");
    TestNode node(io_context, nh);

    io_context.run();
    std::cout << "Execution finished" << std::endl;

    ros::shutdown();
    return 0;
}

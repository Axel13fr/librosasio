#include "librosasio/asio_callbackqueue.h"
#include <ros/ros.h>
#include <thread>
#include <future>

namespace ros
{
extern CallbackQueuePtr g_global_queue;
}

AsioCallbackQueue::AsioCallbackQueue(boost::asio::io_service &io_context)
    : m_io_context(io_context), m_signals(io_context, SIGINT, SIGTERM)
{
  m_signals.async_wait(boost::bind(&AsioCallbackQueue::termSignalHandler, this, _1, _2));
}

AsioCallbackQueue::~AsioCallbackQueue()
{
}

void AsioCallbackQueue::addCallback(const ros::CallbackInterfacePtr &callback, uint64_t owner_id)
{
  ros::CallbackQueue::addCallback(callback, owner_id);
#if BOOST_VERSION <= 106501
  m_io_context.post([this /*,&spin_finished_promise*/]
#else
  boost::asio::post(m_io_context, [this /*,&spin_finished_promise*/]
#endif
                    {
                      // This post() will be called once per callback to process so only process one at a time
                      callOne();
                    });
}

void AsioCallbackQueue::replaceGlobalQueue(boost::asio::io_service &io_context)
{
  ros::g_global_queue.reset(new AsioCallbackQueue(io_context));
}

void AsioCallbackQueue::termSignalHandler(const boost::system::error_code &error, int signal_number)
{
  if (!error)
  {
    switch (signal_number)
    {
      case SIGINT:
      case SIGTERM:
        // set flag to shutdown ros ; this is needed if for example a nodehandle is waiting for the rosmaster
        // TODO: does't work...
        ros::requestShutdown();
        m_io_context.stop();
        break;
      default:
        ROS_DEBUG_STREAM("Received signal" << signal_number);
    }
  }
}

# librosasio [![Build Status](https://travis-ci.com/Axel13fr/librosasio.svg?branch=master)](https://travis-ci.com/Axel13fr/librosasio)

## Description

Library that merges the ROS and Boost.Asio event loops in a single thread. No threading required, best effort latency on callback execution.


# Usage
In your main.cpp init the node in the following way :

```c++
int main(int _argc, char ** _argv)
{
  // Boost Event loop handler
#if BOOST_VERSION <= 106600
  // Version 1.65 on ROS Melodic
  boost::asio::io_service io_context;
#else
  // Version 1.71 on ROS Noetic
  boost::asio::io_context io_context;
#endif

  // Static Function Call doing the magic to unify event loops
  AsioCallbackQueue::replaceGlobalQueue(io_context);
  ros::init(argc, argv, "test_node", ros::init_options::NoSigintHandler);

  ros::NodeHandle nh("TestNode");
  TestNode node(io_context,nh);

  io_context.run();
  ros::shutdown();
  return 0;
}

```

## Examples
* [Simple example](examples/timers)

# Motivations
The boost asio library can be useful for ROS nodes using network sockets,timers...

Unfortunatly both boost asio and ROS need to have their own event loop spinning : `ros::spin()` for ROS, `io_service.run()` for boost Asio.

Multiple methods can be used to take care of both event loops :

# Alternatives
## Polling using a timer

Use the boost event loop and have a boost timer call cyclically `ros::spinOnce()`.
However this method reduces the application reactivity to the frequency of the `pollingTimer`.

## Spawning another thread

Use one thread for each event loop. Using this method the callbacks from boost Asio and from ROS are called without any latency.
The ROS callbacks are called from another thread, so we have to take care to protect any shared ressources with locking mechanisms like mutexes or mailboxes.

# Known issues

Using this library with ROS Kinetic and stopping it via CTRL+C will make the node crash instead of stopping cleanly.
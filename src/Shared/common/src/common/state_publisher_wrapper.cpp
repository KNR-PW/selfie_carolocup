#pragma once

#include <common/python_serializer.hpp>
#include <common/state_publisher.h>

class StatePublisherWrapper : public StatePublisher
{
public:
  using StatePublisher::StatePublisher;
  using StatePublisher::updateState;

public:
  // ROS msgs need to be seralized
  void updateState(const std::string& new_state)
  {
    StatePublisher::updateState(from_python<std_msgs::Int8>(new_state));
  }
};

// overloaded functions service
// static member function pointers
void (StatePublisherWrapper::*updateState_1)(int8_t) = &StatePublisherWrapper::updateState;
void (StatePublisherWrapper::*updateState_2)(const std::string&) = &StatePublisherWrapper::updateState;

BOOST_PYTHON_MODULE(StatePublisherWrapper)
{
  boost::python::class_<StatePublisherWrapper>("StatePublisherWrapper", boost::python::init<const std::string&>())
      .def(boost::python::init<const std::string&, float>())
      .def("updateState", updateState_1)
      .def("updateState", updateState_2);
}
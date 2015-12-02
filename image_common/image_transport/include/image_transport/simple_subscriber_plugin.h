#ifndef IMAGE_TRANSPORT_SIMPLE_SUBSCRIBER_PLUGIN_H
#define IMAGE_TRANSPORT_SIMPLE_SUBSCRIBER_PLUGIN_H

#include "image_transport/subscriber_plugin.h"
#include <boost/scoped_ptr.hpp>

namespace image_transport {

/**
 * \brief Base class to simplify implementing most plugins to Subscriber.
 *
 * The base class simplifies implementing a SubscriberPlugin in the common case that
 * all communication with the matching PublisherPlugin happens over a single ROS
 * topic using a transport-specific message type. SimpleSubscriberPlugin is templated
 * on the transport-specific message type.
 *
 * A subclass need implement only two methods:
 * - getTransportName() from SubscriberPlugin
 * - internalCallback() - processes a message and invoked the user Image callback if
 * appropriate.
 *
 * For access to the parameter server and name remappings, use nh().
 *
 * getTopicToSubscribe() controls the name of the internal communication topic. It
 * defaults to \<base topic\>/\<transport name\>.
 */
template <class M>
class SimpleSubscriberPlugin : public SubscriberPlugin
{
public:
  virtual ~SimpleSubscriberPlugin() {}

  virtual std::string getTopic() const
  {
    if (simple_impl_) return simple_impl_->sub_.getTopic();
    return std::string();
  }

  virtual uint32_t getNumPublishers() const
  {
    if (simple_impl_) return simple_impl_->sub_.getNumPublishers();
    return 0;
  }

  virtual void shutdown()
  {
    if (simple_impl_) simple_impl_->sub_.shutdown();
  }

protected:
  /**
   * \brief Process a message. Must be implemented by the subclass.
   *
   * @param message A message from the PublisherPlugin.
   * @param user_cb The user Image callback to invoke, if appropriate.
   */
  virtual void internalCallback(const typename M::ConstPtr& message, const Callback& user_cb) = 0;

  /**
   * \brief Return the communication topic name for a given base topic.
   *
   * Defaults to \<base topic\>/\<transport name\>.
   */
  virtual std::string getTopicToSubscribe(const std::string& base_topic) const
  {
    return base_topic + "/" + getTransportName();
  }
  
  virtual void subscribeImpl(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                             const Callback& callback, const ros::VoidPtr& tracked_object,
                             const TransportHints& transport_hints)
  {
    // Push each group of transport-specific parameters into a separate sub-namespace
    ros::NodeHandle param_nh(transport_hints.getParameterNH(), getTransportName());
    simple_impl_.reset(new SimpleSubscriberPluginImpl(param_nh));

    simple_impl_->sub_ = nh.subscribe<M>(getTopicToSubscribe(base_topic), queue_size,
                                         boost::bind(&SimpleSubscriberPlugin::internalCallback, this, _1, callback),
                                         tracked_object, transport_hints.getRosHints());
  }

  /**
   * \brief Returns the ros::NodeHandle to be used for parameter lookup.
   */
  const ros::NodeHandle& nh() const
  {
    return simple_impl_->param_nh_;
  }

private:
  struct SimpleSubscriberPluginImpl
  {
    SimpleSubscriberPluginImpl(const ros::NodeHandle& nh)
      : param_nh_(nh)
    {
    }
    
    const ros::NodeHandle param_nh_;
    ros::Subscriber sub_;
  };
  
  boost::scoped_ptr<SimpleSubscriberPluginImpl> simple_impl_;
};

} //namespace image_transport

#endif

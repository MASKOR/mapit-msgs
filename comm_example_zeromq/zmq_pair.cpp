#include "zmq_pair.h"

ZMQPair::ZMQPair()
{
  context_ = new zmq::context_t(1);
  socket_ = new zmq::socket_t(*context_, ZMQ_PAIR);
  connected_ = false;
}

ZMQPair::~ZMQPair()
{
  delete(socket_);
  delete(context_);
}

void
ZMQPair::connect(std::string com)
{
  if (connected_) {
    // throw
  }

  socket_->connect(com);
  connected_ = true;
}

void
ZMQPair::bind(std::string com)
{
  if (connected_) {
    // throw
  }

  socket_->bind(com);
  connected_ = true;
}

std::shared_ptr<std::string>
ZMQPair::receive()
{
  if ( ! connected_) {
    // throw
  }

  zmq::message_t msg_zmq;
  socket_->recv( &msg_zmq );

  std::shared_ptr<std::string> msg( new std::string( static_cast<char*>(msg_zmq.data()), msg_zmq.size() ) );

  return msg;
}

void
ZMQPair::send(std::shared_ptr<std::string> msg)
{
  if ( ! connected_) {
    // throw
  }

  zmq::message_t msg_zmq( msg->size() );
  memcpy (msg_zmq.data(), msg->c_str(), msg->size());
  socket_->send (msg_zmq);
}

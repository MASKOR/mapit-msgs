#ifndef ZMQPAIR_H
#define ZMQPAIR_H

#include <zmq.hpp>
#include <memory>

class ZMQPair
{
private:
  zmq::context_t * context_;
  zmq::socket_t * socket_;
  bool connected_;
public:
  ZMQPair();
  ~ZMQPair();

  void connect(std::string com);
  void bind(std::string com);
  std::shared_ptr<std::string> receive();
  void send(std::shared_ptr<std::string> msg);
};

#endif // ZMQPAIR_H

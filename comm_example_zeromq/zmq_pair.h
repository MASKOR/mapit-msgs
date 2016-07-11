#ifndef ZMQPAIR_H
#define ZMQPAIR_H

#include <google/protobuf/message.h>
#include <zmq.hpp>
#include <memory>
#include <map>

class ZMQPair
{
private:
  zmq::context_t * context_;
  zmq::socket_t * socket_;
  bool connected_;

  typedef std::pair<uint16_t, uint16_t> KeyType;
  typedef std::map<KeyType, google::protobuf::Message *> TypeMap;
  TypeMap message_by_comp_type_;

  KeyType key_from_desc(const google::protobuf::Descriptor *desc);
  std::shared_ptr< ::google::protobuf::Message> new_message_for(uint16_t component_id, uint16_t msg_type);
public:
  ZMQPair();
  ~ZMQPair();

  void connect(std::string com);
  void bind(std::string com);
  std::shared_ptr<google::protobuf::Message> receive();
  void send(std::unique_ptr< ::google::protobuf::Message> msg);

  template <class MT>
  typename std::enable_if<std::is_base_of<google::protobuf::Message, MT>::value, void>::type
  add_message_type()
  {
    MT m;
    const google::protobuf::Descriptor *desc = m.GetDescriptor();
    KeyType key = key_from_desc(desc);
    if (message_by_comp_type_.find(key) != message_by_comp_type_.end()) {
      std::string msg = "Message type " + std::to_string(key.first) + ":" + std::to_string(key.second) + " already registered";
      throw std::runtime_error(msg);
    }
    MT *new_m = new MT();
    message_by_comp_type_[key] = new_m;
  }
};

#endif // ZMQPAIR_H

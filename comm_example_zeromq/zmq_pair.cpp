#include "zmq_pair.h"
#include "cs_comm.pb.h"

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

std::shared_ptr< ::google::protobuf::Message>
ZMQPair::receive()
{
  if ( ! connected_) {
    // throw
  }

  // receive header
  upns::Header h;
  zmq::message_t msg_h;
  socket_->recv( &msg_h );
  h.ParseFromArray(msg_h.data(), msg_h.size());

  // receive msg
  zmq::message_t msg_zmq;
  socket_->recv( &msg_zmq );

  std::shared_ptr<google::protobuf::Message> msg = new_message_for(h.comp_id(), h.msg_type());
  msg->ParseFromArray(msg_zmq.data(), msg_zmq.size());

  return msg;
}

void
ZMQPair::send(std::unique_ptr< ::google::protobuf::Message> msg)
{
  if ( ! connected_) {
    // throw
  }

  // check for COMP_ID and MSG_TYPE
  const google::protobuf::Descriptor *desc = msg->GetDescriptor();
  KeyType key = key_from_desc(desc);
  int comp_id = key.first;
  int msg_type = key.second;

  // send header
  upns::Header h;
  h.set_comp_id(comp_id);
  h.set_msg_type(msg_type);

  int size = h.ByteSize();
  zmq::message_t msg_h( size );
  h.SerializeToArray(msg_h.data(), size);
  socket_->send(msg_h);

  // send msg
  size = msg->ByteSize();
  zmq::message_t msg_zmq( size );
  msg->SerializeToArray(msg_zmq.data(), size);
  socket_->send(msg_zmq);
}

ZMQPair::KeyType
ZMQPair::key_from_desc(const google::protobuf::Descriptor *desc)
{
  const google::protobuf::EnumDescriptor *enumdesc = desc->FindEnumTypeByName("CompType");
  if (! enumdesc) {
    throw std::logic_error("Message does not have CompType enum");
  }
  const google::protobuf::EnumValueDescriptor *compdesc =
    enumdesc->FindValueByName("COMP_ID");
  const google::protobuf::EnumValueDescriptor *msgtdesc =
    enumdesc->FindValueByName("MSG_TYPE");
  if (! compdesc || ! msgtdesc) {
    throw std::logic_error("Message CompType enum hs no COMP_ID or MSG_TYPE value");
  }
  int comp_id = compdesc->number();
  int msg_type = msgtdesc->number();
  if (comp_id < 0 || comp_id > std::numeric_limits<uint16_t>::max()) {
    throw std::logic_error("Message has invalid COMP_ID");
  }
  if (msg_type < 0 || msg_type > std::numeric_limits<uint16_t>::max()) {
    throw std::logic_error("Message has invalid MSG_TYPE");
  }
  return KeyType(comp_id, msg_type);
}

std::shared_ptr<google::protobuf::Message>
ZMQPair::new_message_for(uint16_t component_id, uint16_t msg_type)
{
  KeyType key(component_id, msg_type);

  if (message_by_comp_type_.find(key) == message_by_comp_type_.end()) {
    std::string msg = "Message type " + std::to_string(component_id) + ":" + std::to_string(msg_type) + " not registered";
    throw std::runtime_error(msg);
  }

  google::protobuf::Message *m = message_by_comp_type_[key]->New();
  return std::shared_ptr<google::protobuf::Message>(m);
}

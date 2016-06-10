
/***************************************************************************
 * client.cpp - 
 *
 *  Created: Mon Mar 04 14:09:00 2013
 *  Copyright  2013-2015  Tim Niemueller [www.niemueller.de]
 *             2016 Tobias Neumann
 ****************************************************************************/

/*  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the authors nor the names of its contributors
 *   may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <protobuf_comm/server.h>

#include <boost/asio.hpp>
#include <boost/date_time.hpp>
#include "cs_comm.pb.h"

using namespace protobuf_comm;

static bool quit = false;

ProtobufStreamServer *server_ = NULL;

void
handle_server_client_connected(ProtobufStreamServer::ClientID client,
                               boost::asio::ip::tcp::endpoint &endpoint)
{
  printf("Client connected\n");
}

void
handle_server_client_disconnected(ProtobufStreamServer::ClientID client,
                   const boost::system::error_code &error)
{
  printf("Client disconnected\n");
}

void
handle_message(protobuf_comm::ProtobufStreamServer::ClientID client,
               uint16_t component_id, uint16_t msg_type,
               std::shared_ptr<google::protobuf::Message> msg)
{
  std::shared_ptr<upns::Request> rq;
  if ((rq = std::dynamic_pointer_cast<upns::Request>(msg))) {

    if ( rq->type() == upns::Request::TEXT_OUTPUT ) {
      printf("Got request \"TEXT_OUTPUT\" from \"%s\", will send no reply\n", rq->sender_name().c_str());
    } else if ( rq->type() == upns::Request::POINTCLOUD ) {
      printf("Got request \"POINTCLOUD\" from \"%s\", will send pointcloud to %u\n", rq->sender_name().c_str(), client);

      upns::PointCloud pc;
      pc.set_pointcloud("Hello Daniel");

      server_->send(client, pc);
    }

  } else {

    printf("RECEIVED UNKNOWN Protobuf msg\n");

  }
}

void
handle_server_client_fail(protobuf_comm::ProtobufStreamServer::ClientID client,
           uint16_t component_id, uint16_t msg_type,
           std::string msg) {
  printf("Receive error\n");
}


int
main(int argc, char **argv)
{
  MessageRegister * message_register = new MessageRegister();
  message_register->add_message_type<upns::Request>();

  server_ = new ProtobufStreamServer(4444, message_register);

  server_->signal_received()
      .connect( handle_message );
  server_->signal_receive_failed()
      .connect( handle_server_client_fail );

  printf("ready to receive requests from upns partners (for 1 hour)...\n");
  sleep(3600);

  delete server_;
  delete message_register;

  // Delete all global objects allocated by libprotobuf
  google::protobuf::ShutdownProtobufLibrary();
}

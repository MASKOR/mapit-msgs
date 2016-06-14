
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

#include <zmq.hpp>
#include <pthread.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include "cs_comm.pb.h"

static bool quit = false;

bool
load_pcd(std::string file_name, pcl::PCLPointCloud2 & point_cloud)
{
  printf("Start to read: %s\n", file_name.c_str());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_name, *cloud) == -1) {
    PCL_ERROR ("Couldn't read file: %s \n", file_name.c_str());
    return false;
  }

  pcl::toPCLPointCloud2(*cloud, point_cloud);

  printf("Done reading file\n");

  return true;
}

void *
socket_callback (void *arg)
{
  printf("received callback\n");

  zmq::context_t *context = (zmq::context_t *) arg;

  zmq::socket_t socket (*context, ZMQ_REP);
  socket.connect ("inproc://workers");

  while (true) {
    //  Wait for next request from client
    zmq::message_t request;
    socket.recv (&request);
    std::cout << "Received request: [" << (char*) request.data() << "]" << std::endl;

    //  Do some 'work'
    sleep (10);

    //  Send reply back to client
    zmq::message_t reply (6);
    memcpy ((void *) reply.data (), "World", 6);
    socket.send (reply);
  }
  return (NULL);
}

int
main(int argc, char **argv)
{
  //  Prepare context and sockets
  zmq::context_t context (1);
  zmq::socket_t socket (context, ZMQ_ROUTER);

  socket.bind("tcp://*:4444");
  zmq::socket_t workers (context, ZMQ_DEALER);
  workers.bind ("inproc://workers");

  //  Launch pool of worker threads
  for (int thread_nbr = 0; thread_nbr < 5; thread_nbr++) {
      pthread_t worker;

      pthread_create (&worker, NULL, socket_callback, (void *) &context);
  }

  printf("ready to receive requests from upns partners ...\n");

  //  Connect work threads to client threads via a queue
  zmq::proxy(static_cast<void *>(socket), static_cast<void *>(workers), nullptr);

//  printf("ready to receive requests from upns partners (for 1 hour)...\n");
//  sleep(3600);

  // Delete all global objects allocated by libprotobuf
  google::protobuf::ShutdownProtobufLibrary();
}

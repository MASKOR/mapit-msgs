
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

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include "zmq_pair.h"

#include "cs_comm.pb.h"
#include "transport.pb.h"

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

//void
//process_client_request(std::string &msg)
//{


//}

int
main(int argc, char **argv)
{
  ZMQPair server;
  server.add_message_type<upns::Request>();

  server.bind("tcp://*:4445");

  while (true) {
    std::shared_ptr< ::google::protobuf::Message> msg = server.receive();

    std::shared_ptr<upns::Request> pb_request;
    if ((pb_request = std::dynamic_pointer_cast<upns::Request>(msg))) {
      printf("is upns::Request\n");

      if ( pb_request->type() == upns::Request::TEXT_OUTPUT ) {
        printf("request: Text\nHERE IT IS!!!\n");
      } else if ( pb_request->type() == upns::Request::POINTCLOUD ) {
        printf("request: Point Cloud\n");

        pcl::PCLPointCloud2 pc;
        if ( load_pcd("example_send.pcd", pc) ) {
          std::unique_ptr<upns::PCLPointCloud2_empty> msg_pc(new upns::PCLPointCloud2_empty);

          msg_pc->mutable_header()->set_seq( pc.header.seq );
          msg_pc->mutable_header()->set_stamp( pc.header.stamp );
          msg_pc->mutable_header()->set_frame_id( pc.header.frame_id );

          msg_pc->set_height( pc.height );
          msg_pc->set_width( pc.width );
          for ( pcl::PCLPointField field : pc.fields ) {
            upns::PCLPointField* pf = msg_pc->add_fields();

            pf->set_name( field.name );
            pf->set_offset( field.offset );
            pf->set_datatype( field.datatype );
            pf->set_count( field.count );
          }
          msg_pc->set_is_bigendian( pc.is_bigendian );
          msg_pc->set_point_step( pc.point_step );
          msg_pc->set_row_step( pc.row_step );
          msg_pc->set_is_dense( pc.is_dense );

          server.send( std::move(msg_pc) );

          server.send_raw(pc.data.data(), pc.data.size());
        }
        printf("Data sendet\n");
      }

    } else {
      printf("unknown proto type\n");
    }
  }

  // Delete all global objects allocated by libprotobuf
  google::protobuf::ShutdownProtobufLibrary();
}

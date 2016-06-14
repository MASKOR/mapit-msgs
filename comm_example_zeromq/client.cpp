
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

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include "cs_comm.pb.h"

//using namespace protobuf_comm;

//static bool quit = false;
//ProtobufStreamClient *client_ = NULL;

//void
//signal_handler(const boost::system::error_code& error, int signum)
//{
//  if (!error) {
//    quit = true;
//  }
//}

//pcl::PCLPointCloud2
//proto_to_pcl2(std::shared_ptr<upns::PCLPointCloud2> msg_pc)
//{
//  pcl::PCLPointCloud2 pc;

//  pc.header.seq = msg_pc->header().seq();
//  pc.header.stamp = msg_pc->header().stamp();
//  pc.header.frame_id = msg_pc->header().frame_id();

//  pc.height = msg_pc->height();
//  pc.width = msg_pc->width();

//  for ( upns::PCLPointField field : msg_pc->fields() ) {
//    pcl::PCLPointField pf;
//    pf.name = field.name();
//    pf.offset = field.offset();
//    pf.datatype = field.datatype();
//    pf.count = field.count();

//    pc.fields.push_back( pf );
//  }

//  pc.is_bigendian = msg_pc->is_bigendian();
//  pc.point_step = msg_pc->point_step();
//  pc.row_step = msg_pc->row_step();
//  pc.is_dense = msg_pc->is_dense();

//  pc.data.insert(pc.data.end(),
//                 msg_pc->data().c_str(),
//                 msg_pc->data().c_str() + msg_pc->data().size()
//  );

//  return pc;
//}

//void
//open_viewer(pcl::PCLPointCloud2 pc)
//{
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::fromPCLPointCloud2(pc, *cloud);
////    pcl::io::savePCDFileBinary("received.pcd", cloud);

//  pcl::visualization::PCLVisualizer viewer("3D Viewer");
//  viewer.setBackgroundColor (0, 0, 0);
//  viewer.addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
//  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
//  viewer.addCoordinateSystem (1.0);
//  viewer.initCameraParameters ();

//  while (!viewer.wasStopped ()) {
//    viewer.spinOnce (100);
//    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//  }
//}

//void
//handle_message(uint16_t comp_id, uint16_t msg_type,
//               std::shared_ptr<google::protobuf::Message> msg)
//{
//  std::shared_ptr<upns::PCLPointCloud2> msg_pc;
//  if ((msg_pc = std::dynamic_pointer_cast<upns::PCLPointCloud2>(msg))) {
//    printf("received PCLPointCloud2, going to display...\n");

//    pcl::PCLPointCloud2 pc = proto_to_pcl2(msg_pc);
//    open_viewer(pc);

//  } else {
//    printf("RECEIVED UNKNOWN Protobuf msg\n");
//  }
//}

int
main(int argc, char **argv)
{
//  client_ = new ProtobufStreamClient();

//  client_->async_connect("127.0.0.1", 4444);
//  while ( ! client_->connected() ) {
//    printf("sleep untill connection is established...\n");
//    sleep(1);
//  }

//  MessageRegister & message_register = client_->message_register();
//  message_register.add_message_type<upns::PCLPointCloud2>();

//  printf("Waiting for beacon from upns FH-AC server...\n");

//  client_->signal_received().connect( handle_message );

//  boost::asio::io_service io_service;
//#if BOOST_ASIO_VERSION >= 100601
//  // Construct a signal set registered for process termination.
//  boost::asio::signal_set signals(io_service, SIGINT, SIGTERM);

//  // Start an asynchronous wait for one of the signals to occur.
//  signals.async_wait(signal_handler);
//#endif

//  upns::Request pb_request;
//  pb_request.set_sender_name("client example");
//  pb_request.set_type(upns::Request::POINTCLOUD);

//  client_->send(pb_request);

//  do {
//    io_service.run();
//    io_service.reset();
//  } while (! quit);

//  delete client_;

//  // Delete all global objects allocated by libprotobuf
//  google::protobuf::ShutdownProtobufLibrary();
}

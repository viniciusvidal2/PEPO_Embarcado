#include <ros/ros.h>
#include <dirent.h>
#include <errno.h>
#include <ostream>
#include <istream>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../msgs/imagem.pb.h"
#include "../msgs/nuvem.pb.h"
#include "google/protobuf/io/coded_stream.h"

#include <zmq.hpp>
#include <zmq_utils.h>

using namespace ImagemMsgProto;
using namespace NuvemMsgProto;
using namespace pcl;
using namespace cv;
using namespace std;
using namespace zmq;

typedef PointXYZRGB PointT;

context_t ctx{1};
void *context;
void *im_sender;
void *cl_sender;

////////////////////////////////////////////////////////////////////////////////////////////////////
void imagemCallback(const sensor_msgs::ImageConstPtr& msg){
    // Converte mensagem
    cv_bridge::CvImagePtr imptr = cv_bridge::toCvCopy(msg, msg->encoding);
    // Reduz a resolucao e passa para jpeg
    Mat im;
    imptr->image.copyTo(im);
    resize(im, im, Size(im.cols/3, im.rows/3));
    vector<uchar> buffer_imagem;
    imencode(".jpg", im, buffer_imagem);
    // Envia pelo socket
    zmq_send(im_sender, buffer_imagem.data(), buffer_imagem.size(), 0);
}
////////////////////////////////////////////////////////////////////////////////////////////////////
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
    // Converter mensagem
    PointCloud<PointT>::Ptr cloud (new PointCloud<PointT>);
    fromROSMsg(*msg, *cloud);
    // Cria objeto do protobuf
    Nuvem cloud_proto;
    string buffer_nuvem;
    cloud_proto.set_name("pepo");
    cloud_proto.set_size(cloud->size());
    PointT cloud_point;
    for(size_t k=0; k<cloud->size(); k++){
      cloud_point = cloud->points[k];

      Nuvem::Ponto *p = cloud_proto.add_pontos();
      p->set_x(cloud_point.x); p->set_y(cloud_point.y); p->set_z(cloud_point.z);
      p->set_r(cloud_point.r); p->set_g(cloud_point.g); p->set_b(cloud_point.b);
    }
    cloud->clear();
    // Serializando a mensagem e enviando
    cloud_proto.SerializeToString(&buffer_nuvem);
    zmq_send(cl_sender, buffer_nuvem.data(), buffer_nuvem.size(), 0);
//    message_t nuvem_zmq(buffer_nuvem.length());
//    memcpy(nuvem_zmq.data(), buffer_nuvem.data(), buffer_nuvem.length());
//    cl_sender.send(nuvem_zmq);
}
////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    ros::init(argc, argv, "encaminha_dados_desktop");
    ros::NodeHandle nh;

    context = zmq_ctx_new();
    cl_sender = zmq_socket(context, ZMQ_PUSH);
    int bind1 = zmq_bind(cl_sender, "tcp://*:5558");
    im_sender = zmq_socket(context, ZMQ_PUSH);
    int bind = zmq_bind(im_sender, "tcp://*:5557");

    // Subscriber para topicos de imagem e de nuvem
    ros::Subscriber im_sub = nh.subscribe("/imagem_obj", 100, imagemCallback);
    ros::Subscriber cl_sub = nh.subscribe("/cloud_obj" , 100, cloudCallback);

    ros::Rate r(2);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }

    ctx.close();
    return 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////////

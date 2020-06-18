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

////////////////////////////////////////////////////////////////////////////////////////////////////
void imagemCallback(const sensor_msgs::ImageConstPtr& msg){
    socket_t im_sender(ctx, ZMQ_PUB);
    im_sender.bind("tcp://*:5557");
    // Converte mensagem
    cv_bridge::CvImagePtr im = cv_bridge::toCvCopy(msg, msg->encoding);
    // Reduz a resolucao e passa para jpeg
    resize(im->image, im->image, Size(im->image.rows/3, im->image.cols/3));
    vector<uchar> buffer_imagem;
    imencode(".jpg", im->image, buffer_imagem);
    message_t img_zmq;
    memcpy(img_zmq.data(), buffer_imagem.data(), buffer_imagem.size());
    im_sender.send(img_zmq);
    im_sender.close();
//    // Cria objeto do Protobuf
//    string buffer_imagem;
//    Imagem img_proto;
//    img_proto.set_height(im->image.rows);
//    img_proto.set_width(im->image.cols);
//    img_proto.set_name("pepo");
//    Vec3b cor;
//    for(int i=0; i<img_proto.height(); i++){
//        for(int j=0; j<img_proto.width(); j++){
//            cor = im->image.at<Vec3b>(Point(j, i));

//            Imagem::Pixel *pix = img_proto.add_pixels();
//            pix->set_b(cor(0));
//            pix->set_g(cor(1));
//            pix->set_r(cor(2));
//            pix->set_u(j);
//            pix->set_v(i);
//        }
//    }
//    // Serializando a mensagem e enviando
//    img_proto.SerializeToString(&buffer_imagem);
//    message_t img_zmq(buffer_imagem.length());
//    memcpy(img_zmq.data(), buffer_imagem.data(), buffer_imagem.length());
//    im_sender.send(img_zmq);
//    im_sender.close();
}
////////////////////////////////////////////////////////////////////////////////////////////////////
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
    socket_t cl_sender(ctx, ZMQ_PUB);
    cl_sender.bind("tcp://*:5558");
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
    message_t nuvem_zmq(buffer_nuvem.length());
    memcpy(nuvem_zmq.data(), buffer_nuvem.data(), buffer_nuvem.length());
    cl_sender.send(nuvem_zmq);
    cl_sender.close();
}
////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    ros::init(argc, argv, "encaminha_dados_desktop");
    ros::NodeHandle nh;

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

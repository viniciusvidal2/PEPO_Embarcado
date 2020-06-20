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
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../msgs/imagem.pb.h"
#include "../msgs/nuvem.pb.h"
#include "google/protobuf/io/coded_stream.h"

#include "../../libraries/include/processcloud.h"

#include <zmq.hpp>
#include <zmq_utils.h>

using namespace ImagemMsgProto;
using namespace NuvemMsgProto;
using namespace pcl;
using namespace cv;
using namespace std;
using namespace zmq;

typedef PointXYZRGB PointT;

void *context;
void *im_sender;
void *cl_sender;

cv_bridge::CvImagePtr imptr;
PointCloud<PointXYZ>::Ptr parcial;
int contador_nuvem = 0;
ProcessCloud *pc;
Mutex m;
bool aquisitar_imagem = true, imagem_ok = false;

////////////////////////////////////////////////////////////////////////////////////////////////////
void imagemCallback(const sensor_msgs::ImageConstPtr& msg){
    // Converte mensagem
    if(aquisitar_imagem)
        imptr = cv_bridge::toCvCopy(msg, msg->encoding);
    imagem_ok = true;
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
    if(imagem_ok){
    // Converter mensagem
    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
    fromROSMsg(*msg, *cloud);
    *parcial += *cloud;
    // A nuvem ainda nao foi acumulada, frizar isso
    aquisitar_imagem = true;
    // Se acumulou o suficiente, trabalhar e encaminhar
    if(contador_nuvem == 50){ // Rapidinho assim mesmo
        m.lock();
        // Injetando cor na nuvem
        PointCloud<PointT>::Ptr cloud_color (new PointCloud<PointT>());
        cloud_color->resize(parcial->size());
        #pragma omp parallel for
        for(size_t i=0; i < cloud_color->size(); i++){
            cloud_color->points[i].r = 0; cloud_color->points[i].g = 0; cloud_color->points[i].b = 0;
            cloud_color->points[i].x = parcial->points[i].x;
            cloud_color->points[i].y = parcial->points[i].y;
            cloud_color->points[i].z = parcial->points[i].z;
        }
        // Poupar memoria da parcial
        parcial->clear();
        // Transformando nuvem para o frame da camera
        pc->transformToCameraFrame(cloud_color);
        // Filtrar profundidade pra nao vir aquilo tudo de coisa
        PassThrough<PointT> pass;
        pass.setInputCloud(cloud_color);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0, 8); // Z metros de profundidade
        pass.filter(*cloud_color);
        // Colorir pontos com calibracao default para visualizacao rapida
        aquisitar_imagem = false;
        Mat temp_im;
        imptr->image.copyTo(temp_im);
        pc->colorCloudWithCalibratedImage(cloud_color, temp_im, 1133.3, 1121.6); // Brio
        aquisitar_imagem = true;
        // Filtrando por voxels e outliers - essa vai para visualizacao
        VoxelGrid<PointT> voxel;
        voxel.setInputCloud(cloud_color);
        voxel.setLeafSize(0.01, 0.01, 0.01);
        voxel.filter(*cloud_color);
        // Zerando contador
        contador_nuvem = 0;
        // Cria objeto do protobuf
        Nuvem cloud_proto;
        string buffer_nuvem;
        cloud_proto.set_name("pepo");
        cloud_proto.set_size(cloud_color->size());
        PointT cloud_point;
        for(size_t k=0; k<cloud_color->size(); k++){
          cloud_point = cloud_color->points[k];

          Nuvem::Ponto *p = cloud_proto.add_pontos();
          p->set_x(cloud_point.x); p->set_y(cloud_point.y); p->set_z(cloud_point.z);
          p->set_r(cloud_point.r); p->set_g(cloud_point.g); p->set_b(cloud_point.b);
        }
        cloud_color->clear();
        // Serializando a mensagem e enviando
        cloud_proto.SerializeToString(&buffer_nuvem);
        zmq_send(cl_sender, buffer_nuvem.data(), buffer_nuvem.size(), 0);
        m.unlock();
    } else {
        contador_nuvem++;
    }
    }
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

    // Dar um tempo pra tudo iniciar
    //sleep(10);

    // Inicia nuvem parcial acumulada a cada passagem do laser
    parcial = (PointCloud<PointXYZ>::Ptr) new PointCloud<PointXYZ>();
    parcial->header.frame_id  = "pepo";

    // Inicia classe de processo de nuvens
    pc = new ProcessCloud("/home/pepo/Desktop");
    
    // Subscriber para topicos de imagem e de nuvem
    ros::Subscriber im_sub = nh.subscribe("/image_obj", 10, imagemCallback);
    ros::Subscriber cl_sub = nh.subscribe("/cloud_obj" , 10, cloudCallback);

    ros::Rate r(2);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////////

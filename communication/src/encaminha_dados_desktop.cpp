#include <ros/ros.h>
#include <dirent.h>
#include <errno.h>
#include <ostream>
#include <istream>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

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

#include "../../libraries/include/processcloud.h"


using namespace pcl;
using namespace cv;
using namespace std;

typedef PointXYZRGB PointT;

cv_bridge::CvImagePtr imptr;
int contador_nuvem = 0;
ProcessCloud *pc;
Mutex m;
bool aquisitar_imagem = true, imagem_ok = false;

// Publisher para a nuvem e imagem
image_transport::Publisher im_pub;
ros::Publisher cl_pub;

// Flag de controle para imagem virtual ou nao: se chamar o callback da nuvem quer dizer que
// estamos capturando objeto, portanto a imagem que vai ser publicada sera a virtual
bool imagem_virtual_usuario = false;

PointCloud<PointXYZ>::Ptr cloud_im_virtual;
vector<PointCloud<PointXYZ>> clouds_im_virtual;

int cont_imagem_virtual = 0;

////////////////////////////////////////////////////////////////////////////////////////////////////
void imagemCallback(const sensor_msgs::ImageConstPtr& msg){
    // Converte mensagem
    if(aquisitar_imagem)
        imptr = cv_bridge::toCvCopy(msg, msg->encoding);
    imagem_ok = true;

    if(!imagem_virtual_usuario){
        // Reduz a resolucao e passa para jpeg
        Mat im;
        resize(imptr->image, im, Size(imptr->image.cols/4, imptr->image.rows/4));

        cv_bridge::CvImage msg_out;
        msg_out.header   = msg->header;
        msg_out.encoding = sensor_msgs::image_encodings::BGR8;
        msg_out.image    = im;
        im_pub.publish(msg_out.toImageMsg());
    }

}
////////////////////////////////////////////////////////////////////////////////////////////////////
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
    imagem_virtual_usuario = true; // Estamos na aquisicao de objeto, transmitir imagem virtual

    if(imagem_ok){
        // Converter mensagem
        PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
        fromROSMsg(*msg, *cloud);
        pc->cleanMisreadPoints(cloud);

        // A todo tempo deve haver imagem virtual, portanto agrupando
        *cloud_im_virtual += *cloud;
        cont_imagem_virtual++;

        if(cont_imagem_virtual == 5){ // Se inteirar 5, ja projeta -> mais rapido a experiencia

            m.lock();

            // Transformando nuvem para o frame da camera
            pc->transformToCameraFrame(cloud_im_virtual);
            // Colorir pontos com calibracao default para visualizacao rapida
            aquisitar_imagem = false;
//            Mat temp_im;
//            imptr->image.copyTo(temp_im);
            // Imagem virtual iniciada
            Mat virtual_image(Size(imptr->image.cols, imptr->image.rows), CV_8UC3, Scalar(0, 0, 0));
            pc->getVirtualImage(cloud_im_virtual, imptr->image, virtual_image, 1);
            resize(virtual_image, virtual_image, Size(320, 180), cv::INTER_CUBIC);
            // Poupar memoria da parcial
            cloud_im_virtual->clear();

            aquisitar_imagem = true;
            m.unlock();

            // Publicar imagem virtual
            cv_bridge::CvImage msg_out;
            msg_out.header   = msg->header;
            msg_out.encoding = sensor_msgs::image_encodings::BGR8;
            msg_out.image    = virtual_image;
            im_pub.publish(msg_out.toImageMsg());

            cont_imagem_virtual = 0; // Reseta para a proxima acumulacao de nuvens
        }
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    ros::init(argc, argv, "imagem_lr_app");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    // Inicia nuvem parcial acumulada a cada passagem do laser
    cloud_im_virtual = (PointCloud<PointXYZ>::Ptr) new PointCloud<PointXYZ>();
    cloud_im_virtual->header.frame_id  = "pepo";

    // Publicadores
    im_pub = it.advertise("/image_user", 10);
    cl_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_user", 10);

    // Inicia classe de processo de nuvens
    pc = new ProcessCloud("/home/pepo/Desktop");

    // Subscriber para topicos de imagem e de nuvem
    ros::Subscriber im_sub = nh.subscribe("/camera/image_raw", 10, imagemCallback);
    ros::Subscriber cl_sub = nh.subscribe("/cloud_virtual_image", 10, cloudCallback);

    ros::spin();

    return 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////////

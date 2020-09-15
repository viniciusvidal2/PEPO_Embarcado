#include <ros/ros.h>
#include <dirent.h>
#include <errno.h>
#include <ostream>
#include <istream>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
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
PointCloud<PointXYZ>::Ptr parcial;
int contador_nuvem = 0;
ProcessCloud *pc;
Mutex m;
bool aquisitar_imagem = true, imagem_ok = false;

// Publisher para a nuvem e imagem
image_transport::Publisher im_pub;
ros::Publisher cl_pub;

////////////////////////////////////////////////////////////////////////////////////////////////////
void imagemCallback(const sensor_msgs::ImageConstPtr& msg){
    // Converte mensagem
    if(aquisitar_imagem)
        imptr = cv_bridge::toCvCopy(msg, msg->encoding);
    imagem_ok = true;
    // Reduz a resolucao e passa para jpeg
    Mat im;
    resize(imptr->image, im, Size(im.cols/4, im.rows/4));
    cv_bridge::CvImage out_msg;
    out_msg.header   = msg->header;
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.image    = im;

    im_pub.publish(out_msg.toImageMsg());
}
////////////////////////////////////////////////////////////////////////////////////////////////////
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
    if(imagem_ok){
        // Converter mensagem
        PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
        fromROSMsg(*msg, *cloud);
        pc->cleanMisreadPoints(cloud);
        *parcial += *cloud;
        // A nuvem ainda nao foi acumulada, frizar isso
        aquisitar_imagem = true;
        // Se acumulou o suficiente, trabalhar e encaminhar
        if(contador_nuvem == 100){ // Rapidinho assim mesmo
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
            // Colorir pontos com calibracao default para visualizacao rapida
            aquisitar_imagem = false;
            Mat temp_im;
            imptr->image.copyTo(temp_im);
            //resize(temp_im, temp_im, Size(temp_im.cols/4, temp_im.rows/4));
            pc->colorCloudWithCalibratedImage(cloud_color, temp_im, 1);
            aquisitar_imagem = true;
            // Zerando contador
            contador_nuvem = 0;
            // Publica para o usuario
            sensor_msgs::PointCloud2 cloud_msg;
            toROSMsg(*cloud_color, cloud_msg);
            cloud_msg.header.frame_id = "map";
            cloud_msg.header.stamp = ros::Time::now();
            cl_pub.publish(cloud_msg);
            cloud_color->clear();

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
    image_transport::ImageTransport it(nh);

    // Inicia nuvem parcial acumulada a cada passagem do laser
    parcial = (PointCloud<PointXYZ>::Ptr) new PointCloud<PointXYZ>();
    parcial->header.frame_id  = "pepo";

    // Publicadores
    im_pub = it.advertise("/image_user", 10);
    cl_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_user", 10);

    // Inicia classe de processo de nuvens
    pc = new ProcessCloud("/home/pepo/Desktop");
    
    // Subscriber para topicos de imagem e de nuvem
    ros::Subscriber im_sub = nh.subscribe("/image_temp", 10, imagemCallback);
    ros::Subscriber cl_sub = nh.subscribe("/cloud_temp", 10, cloudCallback);

    ros::Rate r(2);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////////

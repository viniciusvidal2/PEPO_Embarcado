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

using namespace cv;
using namespace std;

cv_bridge::CvImagePtr imptr;
Mutex m;
bool aquisitar_imagem = true, imagem_ok = false;

// Publisher para a nuvem e imagem
image_transport::Publisher im_pub;

////////////////////////////////////////////////////////////////////////////////////////////////////
void imagemCallback(const sensor_msgs::ImageConstPtr& msg){
    // Converte mensagem
    if(aquisitar_imagem)
        imptr = cv_bridge::toCvCopy(msg, msg->encoding);
    imagem_ok = true;
    // Reduz a resolucao e passa para jpeg
    Mat im;
    resize(imptr->image, im, Size(imptr->image.cols/4, imptr->image.rows/4));
    cv_bridge::CvImage msg_out;
    msg_out.header   = msg->header;
    msg_out.encoding = sensor_msgs::image_encodings::BGR8;
    msg_out.image    = im;

    im_pub.publish(msg_out.toImageMsg());
}
////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    ros::init(argc, argv, "imagem_lr_app");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    // Publicadores
    im_pub = it.advertise("/image_user", 10);
    
    // Subscriber para topicos de imagem e de nuvem
    ros::Subscriber im_sub = nh.subscribe("/camera/image_raw", 10, imagemCallback);

    while(ros::ok())
        ros::spinOnce();

    return 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////////

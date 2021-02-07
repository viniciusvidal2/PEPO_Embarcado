#ifndef PROCESSCLOUD_H
#define PROCESSCLOUD_H

#include "ros/ros.h"

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/videoio.hpp"
#include <opencv2/calib3d.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl_ros/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <string>
#include <iostream>
#include <ros/ros.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <cstdlib>
#include <csignal>
#include <ctime>
#include <math.h>

/// Essa classe tera todas as funçoes para trabalhar a nuvem de pontos, calcular normais, colorir,
/// salvar nuvens e imagens e ate mesmo projetar os pontos para criar a imagem de laser virtual
using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace cv;
using namespace Eigen;

typedef PointXYZRGB       PointT ;
typedef PointXYZRGBNormal PointTN;

class ProcessCloud
{
public:
  ProcessCloud(string _pasta);
  virtual ~ProcessCloud();
  void transformToCameraFrame(PointCloud<PointT>::Ptr nuvem);
  void transformToCameraFrame(PointCloud<PointXYZ>::Ptr nuvem);
  void transformCloudAndCamServoAngles(PointCloud<PointT>::Ptr cloud, float pan, float tilt, Vector3f &C, Quaternion<float> &q);
  void colorCloudWithCalibratedImage(PointCloud<PointT>::Ptr cloud_in, Mat image, float scale);
  void getVirtualImage(PointCloud<PointXYZ>::Ptr cloud_in, Mat image, Mat &iv, float scale);

  void saveCloud(PointCloud<PointT>::Ptr nuvem, string nome);
  void saveCloud(PointCloud<PointXYZ>::Ptr nuvem, string nome);
  string getFolderName();
  void setFolderName(string name);
  Matrix3f euler2matrix(float r, float p, float y);
  void cleanMisreadPoints(PointCloud<PointXYZ>::Ptr cloud);
  Vector2f getFocuses(int scale);

  // Entradas apos colocar a fog embarcada
  void blueprint(PointCloud<PointT>::Ptr cloud_in, float sa, float sr, Mat &bp);
  string escreve_linha_sfm(string nome, Matrix3f r, Vector3f t);
  void compileFinalSFM(vector<string> linhas);
  Vector3f gettCam();

private:

  /// Variaveis
  std::string pasta;     // Nome da pasta a salvar as coisas
  Matrix3f K1;
  MatrixXf Rt1;
  Matrix3f K4;
  MatrixXf Rt4;

};

#endif // PROCESSCLOUD_H

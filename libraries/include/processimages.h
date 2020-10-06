#ifndef PROCESSIMAGES_H
#define PROCESSIMAGES_H

#include "ros/ros.h"

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/videoio.hpp"
#include <opencv2/calib3d.hpp>

#include <pcl_conversions/pcl_conversions.h>

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

using namespace std;
using namespace cv;
using namespace Eigen;

class ProcessImages
{
public:
  ProcessImages(string _pasta);
  virtual ~ProcessImages();

  void saveImage(cv::Mat img, std::string nome);
  string getFolderName();
  void setFolderName(string name);

  void estimateRaw360(vector<Quaternion<float>> qs, Vector2f fs);

private:

  void doTheThing(float sd, Vector3f p2, Vector3f p4, Vector3f p5, Mat im, Mat &im360);

  /// Variaveis
  std::string pasta;     // Nome da pasta a salvar as coisas

};

#endif // PROCESSCLOUD_H

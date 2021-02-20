#ifndef DYNAMIXELSERVOS_H
#define DYNAMIXELSERVOS_H

#include "ros/ros.h"

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <string>
#include <iostream>
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
using namespace Eigen;

class DynamixelServos
{
public:

  DynamixelServos();
  virtual ~DynamixelServos();

  /// Metodos
  int deg2raw(float deg, string motor);
  float raw2deg(int raw, string motor);

  /// Variaveis
  float raw_min_pan, raw_max_pan;
  float raw_min_tilt, raw_hor_tilt, raw_max_tilt;

  float raw_deg_pan, deg_raw_pan;
  float raw_deg_tilt, deg_raw_tilt;

  float deg_min_pan, deg_max_pan;
  float deg_min_tilt, deg_hor_tilt, deg_max_tilt;

private:

  float deg_bottom_tilt, deg_up_tilt;
  float raw_bottom_tilt, raw_up_tilt;

};

#endif // DYNAMIXELSERVOS_H

/// Includes
///
#include <iostream>
#include <ros/ros.h>
#include <ros/service_server.h>
#include <string>
#include <math.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

#include "../../libraries/include/processcloud.h"
#include "pepo/comandoObj.h"

/// Namespaces
///
using namespace pcl;
using namespace pcl::io;
using namespace cv;
using namespace std;

/// Definiçoes
///
typedef PointXYZRGB PointT;

/// Variaveis Globais
///
cv_bridge::CvImagePtr image_ptr; // Ponteiro para imagem da camera
bool aquisitando = false, aquisitar_imagem = false, fim_processo = false;
int contador_nuvem = 0, N = 300; // Quantas nuvens aquisitar em cada parcial
// Classe de processamento de nuvens
ProcessCloud *pc;
// Nuvem de pontos parciais
PointCloud<PointXYZ>::Ptr parcial;
// Testando sincronizar subscribers por mutex
Mutex m;
// Contador de aquisicoes - usa tambem para salvar no lugar certo
int cont_aquisicao = 0;

/// Callback da camera
///
void camCallback(const sensor_msgs::ImageConstPtr& msg){
    // Aqui ja temos a imagem em ponteiro de opencv, depois de pegar uma desabilitar
    if(aquisitar_imagem)
        image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
}

/// Callback do laser
///
void laserCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    if(aquisitando){
        // Ler a mensagem e acumular na nuvem total por N vezes
        PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>());
        fromROSMsg (*msg, *cloud);
        *parcial += *cloud;
        // A nuvem ainda nao foi acumulada, frizar isso
        aquisitar_imagem = true;
        // Se total acumulado, travar o resto e trabalhar
        if(contador_nuvem == N){
            m.lock();
            ROS_WARN("Aquisicao %d foi acumulada, processando ...", cont_aquisicao);
            // Vira a variavel de controle de recebimento de imagens e da nuvem
            aquisitar_imagem = false;
            aquisitando = false;
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
            pass.setFilterLimits(0, 10); // Z metros de profundidade
            pass.filter(*cloud_color);
            // Colorir pontos com calibracao default para visualizacao rapida
            ROS_WARN("Colorindo nuvem para salvar com parametros default ...");
            pc->colorCloudWithCalibratedImage(cloud_color, image_ptr->image, 1133.3, 1121.6); // Brio
            // Filtrando por voxels e outliers - essa vai para visualizacao
            ROS_WARN("Filtrando nuvem ...");
            VoxelGrid<PointT> voxel;
            voxel.setInputCloud(cloud_color);
            voxel.setLeafSize(0.01, 0.01, 0.01);
            voxel.filter(*cloud_color);
            // Salvar dados parciais na pasta Dados_PEPO (ou o nome inserido), no Desktop
            ROS_WARN("Salvando dados de imagem e nuvem da aquisicao %d ...", cont_aquisicao);
            std::string nome_imagem_atual = "imagem_"+std::to_string(cont_aquisicao);
            pc->saveImage(image_ptr->image, nome_imagem_atual);
            pc->saveCloud(cloud_color, "pf_"+std::to_string(cont_aquisicao));
            //////////////////////
            // Zerar contador de nuvens da parcial
            contador_nuvem = 0;
            ROS_WARN("Terminada aquisicao da nuvem %d", cont_aquisicao);
        } else {
            contador_nuvem++;
        }
    }
}

/// Servico para controle de aquisicao
///
bool comando_proceder(pepo::comandoObj::Request &req, pepo::comandoObj::Response &res){
    if(req.comando == 1){ // Havera mais uma nova aquisicao
        aquisitando = true;
        cont_aquisicao++;
        aquisitar_imagem = true;
        res.result = 1;
        ROS_INFO("Realizando aquisicao na posicao %d ...", cont_aquisicao);
    } else if (req.comando == 2) { // Acabamos de aquisitar
        fim_processo = true;
        res.result = 1;
        ROS_INFO("Finalizando o processo ...");
    } else {
        res.result = 0;
    }

    return true;
}

/// Main
///
int main(int argc, char **argv)
{
  ros::init(argc, argv, "scanner_obj");
  ros::NodeHandle nh;
  ros::NodeHandle n_("~");
  ROS_INFO("Iniciando o processo do SCANNER de objeto ...");

  // Pegando o nome da pasta por parametro
  string nome_param;
  n_.param("pasta", nome_param, string("Dados_PEPO"));

  // Apagando pasta atual e recriando a mesma na area de trabalho
  char* home;
  home = getenv("HOME");
  std::string pasta = std::string(home)+"/Desktop/"+nome_param.c_str()+"/";
  system(("rm -r "+pasta).c_str());
  mkdir(pasta.c_str(), 0777);

  // Inicia nuvem parcial acumulada a cada passagem do laser
  parcial  = (PointCloud<PointXYZ>::Ptr) new PointCloud<PointXYZ>();
  parcial->header.frame_id  = "pepo";

  // Inicia classe de processo de nuvens
  pc = new ProcessCloud(pasta);

  // Inicia servidor que recebe o comando sobre como proceder com a aquisicao
  ros::ServiceServer procedimento = nh.advertiseService("/proceder_obj", comando_proceder);

  // Subscribers dessincronizados para mensagens de laser, imagem e motores
  ros::Subscriber sub_laser = nh.subscribe("/livox/lidar"     , 1, laserCallback);
  ros::Subscriber sub_cam   = nh.subscribe("/camera/image_raw", 1, camCallback  );

  ROS_INFO("Comecando a aquisicao ...");

  ros::Rate r(2);
  while(ros::ok()){
      r.sleep();
      ros::spinOnce();

      if(fim_processo){
        ros::shutdown();
        break;
      }
  }

  return 0;
}

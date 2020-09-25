/// Includes
///
#include <iostream>
#include <ros/ros.h>
#include <ros/service_server.h>
#include <string>
#include <math.h>

#include <pcl/filters/passthrough.h>

#include "../../libraries/include/processcloud.h"
#include "../../libraries/include/processimages.h"
#include "pepo_obj/comandoObj.h"

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
int contador_nuvem = 0, N = 400; // Quantas nuvens aquisitar em cada parcial
// Classe de processamento de nuvens
ProcessCloud *pc;
ProcessImages *pi;
// Nuvem de pontos parciais
PointCloud<PointXYZ>::Ptr parcial;
// Testando sincronizar subscribers por mutex
Mutex m;
// Contador de aquisicoes - usa tambem para salvar no lugar certo
int cont_aquisicao = 0;
// Publisher para a nuvem e imagem
ros::Publisher im_pub;
ros::Publisher cl_pub;

/// Callback da camera
///
void camCallback(const sensor_msgs::ImageConstPtr& msg){
    // Aqui ja temos a imagem em ponteiro de opencv, depois de pegar uma desabilitar
    if(aquisitar_imagem)
        image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    // Publicando a imagem para ver o no de comunicacao com o desktop
    im_pub.publish(*msg);
}

/// Callback do laser
///
void laserCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    // Publicar a nuvem de pontos para o no de comunicacao com o Desktop
    cl_pub.publish(*msg);
    if(aquisitando){
        // Ler a mensagem e acumular na nuvem total por N vezes
        PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>());
        fromROSMsg (*msg, *cloud);
        pc->cleanMisreadPoints(cloud);
        *parcial += *cloud;
        // A nuvem ainda nao foi acumulada, frizar isso
        aquisitar_imagem = true;
        // Se total acumulado, travar o resto e trabalhar
        if(contador_nuvem == N){
            cont_aquisicao++;
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
            pass.setFilterLimits(0, 80); // Z metros de profundidade
            pass.filter(*cloud_color);
            // Colorir pontos com calibracao default para visualizacao rapida
            ROS_WARN("Colorindo nuvem para salvar com parametros default ...");
            pc->colorCloudWithCalibratedImage(cloud_color, image_ptr->image, 1); // Brio
            // Filtrando por voxels e outliers - essa vai para visualizacao
            //ROS_WARN("Filtrando nuvem ...");
            //VoxelGrid<PointT> voxel;
            //voxel.setInputCloud(cloud_color);
            //voxel.setLeafSize(0.01, 0.01, 0.01);
            //voxel.filter(*cloud_color);
            // Salvar dados parciais na pasta Dados_PEPO (ou o nome inserido), no Desktop
            ROS_WARN("Salvando dados de imagem e nuvem da aquisicao %d ...", cont_aquisicao);
            if(cont_aquisicao < 10){
              pi->saveImage(image_ptr->image, "imagem_00"+std::to_string(cont_aquisicao));
              pc->saveCloud(cloud_color, "pf_00"+std::to_string(cont_aquisicao));
            } else if(cont_aquisicao < 100) {
              pi->saveImage(image_ptr->image, "imagem_0"+std::to_string(cont_aquisicao));
              pc->saveCloud(cloud_color, "pf_0"+std::to_string(cont_aquisicao));
            } else {
              pi->saveImage(image_ptr->image, "imagem_"+std::to_string(cont_aquisicao));
              pc->saveCloud(cloud_color, "pf_"+std::to_string(cont_aquisicao));
            }
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
bool comando_proceder(pepo_obj::comandoObj::Request &req, pepo_obj::comandoObj::Response &res){
    if(req.comando == 1){ // Havera mais uma nova aquisicao
        aquisitando = true;
        aquisitar_imagem = true;
        res.result = 1;
        ROS_INFO("Realizando aquisicao na posicao %d ...", cont_aquisicao+1);
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
  parcial = (PointCloud<PointXYZ>::Ptr) new PointCloud<PointXYZ>();
  parcial->header.frame_id  = "pepo";

  // Inicia classe de processo de nuvens
  pc = new ProcessCloud(pasta);
  pi = new ProcessImages(pasta);

  // Inicia servidor que recebe o comando sobre como proceder com a aquisicao
  ros::ServiceServer procedimento = nh.advertiseService("/proceder_obj", comando_proceder);

  // Publicadores
  im_pub = nh.advertise<sensor_msgs::Image      >("/image_temp", 10);
  cl_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_temp", 10);

  // Subscribers dessincronizados para mensagens de laser, imagem e motores
  ros::Subscriber sub_laser = nh.subscribe("/livox/lidar"     , 10, laserCallback);
  ros::Subscriber sub_cam   = nh.subscribe("/camera/image_raw", 10, camCallback  );

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

/// Includes
///
#include <iostream>
#include <ros/ros.h>
#include <string>
#include <math.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <dynamixel_workbench_msgs/DynamixelCommand.h>
#include <dynamixel_workbench_msgs/DynamixelInfo.h>
#include <dynamixel_workbench_msgs/JointCommand.h>
#include <dynamixel_workbench_msgs/DynamixelState.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "../../libraries/include/processcloud.h"
#include "led_control/LED.h"

/// Namespaces
///
using namespace pcl;
using namespace pcl::io;
using namespace cv;
using namespace std;
using namespace message_filters;

/// Definiçoes
///
typedef PointXYZRGB PointT;
typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> syncPolicy;

/// Variaveis globais
///
// Vetores para posicao angular dos servos
vector<int>   pans_raw, tilts_raw; // [RAW]
vector<float> pans_deg, tilts_deg; // [DEG]
// Limites em raw e deg para os servos de pan e tilt
double raw_min_pan = 35, raw_max_pan = 4077;
double deg_min_pan =  3, deg_max_pan =  358;
float raw_min_tilt = 2595, raw_hor_tilt = 2280, raw_max_tilt = 1595  ;
float deg_min_tilt =   28, deg_hor_tilt =    0, deg_max_tilt =  -60.9;
float raw_deg = 4096.0f/360.0f, deg_raw = 1/raw_deg;
// Servico para mover os servos
ros::ServiceClient comando_motor;
dynamixel_workbench_msgs::JointCommand cmd;
// Servico para controlar o LED
ros::ServiceClient comando_leds;
// Posicao atual de aquisicao
int indice_posicao = 0;
// Raio de aceitacao de posicao angular
int dentro = 5; // [RAW]
// Flags de controle
bool aquisitar_imagem = false, mudando_vista_pan = false, processar_nuvem_e_enviar = false, iniciar_laser = false;
// Publicador de imagem, nuvem parcial e odometria
ros::Publisher cl_pub;
ros::Publisher od_pub;
ros::Publisher an_pub;
// Classe de processamento de nuvens
ProcessCloud *pc;
// Nuvens de pontos e vetor de nuvens parciais
PointCloud<PointXYZ>::Ptr parcial;
// Valor do servo naquele instante em variavel global para ser acessado em varios callbacks
int pan, tilt;
// Vetor de imagens em baixa resolucao
vector<Mat> imagens_baixa_resolucao;
// Vetor de odometrias para cada imagem - somente tilt ja devia ser suficiente
vector<int> tilts_imagens_pan_atual;
// Quantos tilts vamos capturar
int ntilts;
// Ponteiro de cv_bridge para a imagem
cv_bridge::CvImagePtr image_ptr;
// Controle de tempo do subscriber do laser
ros::Time tempo_laser;

///////////////////////////////////////////////////////////////////////////////////////////
int deg2raw(double deg, string motor){
    if(motor == "pan")
        return int((deg - deg_min_pan )*raw_deg + raw_min_pan);
    else
        return int((deg - deg_min_tilt)*raw_deg + raw_min_tilt);
}
float raw2deg(int raw, string motor){
    if(motor == "pan")
        return (float(raw) - raw_min_pan )*deg_raw + deg_min_pan;
    else
        return (float(raw) - raw_min_tilt)*deg_raw + deg_min_tilt;
}
///////////////////////////////////////////////////////////////////////////////////////////

/// Callback da camera
///
void camCallback(const sensor_msgs::ImageConstPtr& msg){
    // Aqui ja temos a imagem em ponteiro de opencv, depois de pegar uma desabilitar
    if(aquisitar_imagem)
        image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
}

/// Callback do laser e servos
///
void laserServosCallback(const sensor_msgs::PointCloud2ConstPtr &msg_cloud, const nav_msgs::OdometryConstPtr &msg_servos){
    cout << "Tempo entre uma mensagem e outra: " << ros::Time::now() - tempo_laser << "   Bytes na mensagem: " << msg_cloud->data.size()*msg_cloud->point_step << endl;
    tempo_laser = ros::Time::now();
    // As mensagens trazem angulos em unidade RAW
    pan = int(msg_servos->pose.pose.position.x), tilt = int(msg_servos->pose.pose.position.y);
    // Se nao estamos processando a nuvem e publicando, nem mudando de vista em pan, captar
    if(!processar_nuvem_e_enviar && !mudando_vista_pan && iniciar_laser){
        // Converter mensagem
        PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
        fromROSMsg(*msg_cloud, *cloud);
        // Transformando para o frame da camera
        pc->transformToCameraFrame(cloud);
        // Aplicar transformada de acordo com o angulo - somente tilt
        float t = raw2deg(tilt, "tilt");
        Matrix3f R = pc->euler2matrix(0, DEG2RAD(-t), 0);
        Matrix4f T = Matrix4f::Identity();
        T.block<3,3>(0, 0) = R;
        transformPointCloud(*cloud, *cloud, T);
        // Acumular nuvem parcial
        *parcial += *cloud;
    }
    cout << "Tempo para processar essa ultima nuvem com odometria:  " << ros::Time::now() - tempo_laser << endl;
    tempo_laser = ros::Time::now();
}

/// Main
///
int main(int argc, char **argv)
{
    ros::init(argc, argv, "scanner_space3");
    ros::NodeHandle nh;
    ros::NodeHandle n_("~");
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
    ROS_INFO("Iniciando o processo do SCANNER - aguardando servos ...");

    // Pegando os parametros
    string nome_param;
    int step = 30; // [DEG]
    n_.param<string>("pasta", nome_param, string("Dados_PEPO"));
    n_.param<int   >("step" , step      , 30                  );

    // Apagando pasta atual e recriando a mesma na area de trabalho
    char* home;
    home = getenv("HOME");
    string pasta = string(home)+"/Desktop/"+nome_param.c_str()+"/";
    system(("rm -r "+pasta).c_str());
    mkdir(pasta.c_str(), 0777);

    /// Preenchendo vetor de pan e tilt primeiro para a camera
    // Pontos de observacao em tilt
    vector<float> tilts_camera_deg {deg_min_tilt, deg_hor_tilt, -30.0f, deg_max_tilt};
    ntilts = tilts_camera_deg.size();
    // Pontos de observacao em pan
    int vistas_pan = int(deg_max_pan - deg_min_pan)/step + 2; // Vistas na horizontal, somar inicio e final do range
    vector<float> pans_camera_deg;
    for(int j=0; j < vistas_pan-1; j++)
        pans_camera_deg.push_back(deg_min_pan + float(j*step));
    pans_camera_deg.push_back(deg_max_pan);
    // Enchendo vetores de waypoints de imagem em deg e raw globais
    for(int j=0; j < pans_camera_deg.size(); j++){
        for(int i=0; i < tilts_camera_deg.size(); i++){
            if(remainder(j, 2) == 0){
                tilts_deg.push_back(tilts_camera_deg[i]);
                tilts_raw.push_back(deg2raw(tilts_camera_deg[i], "tilt"));
            } else {
                tilts_deg.push_back(tilts_camera_deg[tilts_camera_deg.size() - 1 - i]);
                tilts_raw.push_back(deg2raw(tilts_camera_deg[tilts_camera_deg.size() - 1 - i], "tilt"));
            }
            pans_deg.push_back(pans_camera_deg[j]);
            pans_raw.push_back(deg2raw(pans_camera_deg[j], "pan"));
        }
    }

    // Inicia servico para mexer os servos
    comando_motor = nh.serviceClient<dynamixel_workbench_msgs::JointCommand>("/joint_command");

    // Ligar o LED piscando rapido - MOTORES SE AJUSTANDO
    led_control::LED cmd_led;
    cmd_led.request.led = 2;
    comando_leds.call(cmd_led);

    // Iniciando ponteiro de imagem em cv_bridge
    image_ptr = (cv_bridge::CvImagePtr) new cv_bridge::CvImage;

    // Subscriber de imagem
    ros::Subscriber sub_cam = nh.subscribe("/camera/image_raw", 10, camCallback);

    // Enviando scanner para o inicio
    cmd.request.unit = "raw";
    cmd.request.pan_pos  = pans_raw[0];
    cmd.request.tilt_pos = tilts_raw[0];

    ros::Rate r(20);
    ROS_ERROR("Esperando a comunicacao com os servos !! ...");
    while(!comando_motor.call(cmd)){
        ros::spinOnce();
        r.sleep();
    }
    ROS_INFO("Servos comunicando e indo para a posicao inicial ...");
    sleep(9); // Esperar os servos pararem de balancar e driver de imagem ligar

    // Inicia classe de processo de nuvens
    pc = new ProcessCloud(pasta);

    // Publicadores
    cl_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_space", 10);
    od_pub = nh.advertise<nav_msgs::Odometry      >("/angle_space", 10);
    an_pub = nh.advertise<nav_msgs::Odometry      >("/poses_space", 10);

    // Iniciando a nuvem parcial acumulada de cada pan
    parcial = (PointCloud<PointXYZ>::Ptr) new PointCloud<PointXYZ>();
    parcial->header.frame_id  = "map";

    // Iniciar subscritor de laser sincronizado com posicao dos servos
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_nuvem (nh, "/livox/lidar"                    , 10);
    message_filters::Subscriber<nav_msgs::Odometry      > sub_odom  (nh, "/dynamixel_angulos_sincronizados", 10);
    Synchronizer<syncPolicy> sync(syncPolicy(10), sub_nuvem, sub_odom);
    message_filters::Connection conexao = sync.registerCallback(boost::bind(&laserServosCallback, _1, _2));

    ROS_INFO("Comecando a aquisicao ...");

    // LED piscando lentamente - AQUISITANDO
    cmd_led.request.led = 3;
    comando_leds.call(cmd_led);

    while(ros::ok()){
        // Se capturamos ja toda a vista em pan, processar e enviar
        if(processar_nuvem_e_enviar){
            // Filtrar nuvem por voxel
            ROS_INFO("Filtrando nuvem parcial ...");
            VoxelGrid<PointXYZ> voxel;
            float ls = 0.02;
            voxel.setLeafSize(ls, ls, ls);
            voxel.setInputCloud(parcial);
            //voxel.filter(*parcial);
            // Colorir nuvem com todas as imagens
            ROS_INFO("Colorindo nuvem parcial ...");
            PointCloud<PointT>::Ptr parcial_color (new PointCloud<PointT>);
            parcial_color->resize(parcial->size());
#pragma omp parallel for
            for(size_t i=0; i<parcial_color->size(); i++){
                parcial_color->points[i].x = parcial->points[i].x; parcial_color->points[i].y = parcial->points[i].y; parcial_color->points[i].z = parcial->points[i].z;
                parcial_color->points[i].r = 200                 ; parcial_color->points[i].g = 200                 ; parcial_color->points[i].b = 200                 ;
            }
            parcial->clear();
            for(int i=0; i<imagens_baixa_resolucao.size(); i++){
                // Transformar a nuvem para o centro de cada camera (em tilt) e depois projetar
                float t = raw2deg(tilts_imagens_pan_atual[i], "tilt");
                Matrix3f R = pc->euler2matrix(0, DEG2RAD(t), 0);
                Matrix4f T = Matrix4f::Identity();
                T.block<3,3>(0, 0) = R;
                transformPointCloud<PointT>(*parcial_color, *parcial_color, T);
                pc->colorCloudWithCalibratedImage(parcial_color, imagens_baixa_resolucao[i], 4);
                transformPointCloud<PointT>(*parcial_color, *parcial_color, T.inverse());
            }
            // Publicar tudo para a fog - nuvem e odometria
            ROS_INFO("Publicando nuvem e odometria ...");
            sensor_msgs::PointCloud2 msg_out;
            toROSMsg(*parcial_color, msg_out);
            msg_out.header.stamp = ros::Time::now();
            msg_out.header.frame_id = "map";
            nav_msgs::Odometry odom_out;
            odom_out.pose.pose.position.x = pan;
            odom_out.pose.pose.position.y = tilt;
            odom_out.pose.pose.position.z = indice_posicao;
            odom_out.pose.pose.orientation.w = pans_raw.size(); // Quantidade total de aquisicoes para o fog ter nocao
            odom_out.header.stamp = msg_out.header.stamp;
            odom_out.header.frame_id = msg_out.header.frame_id;
            od_pub.publish(odom_out);
            cl_pub.publish(msg_out);
            // Zerando parcial para proxima vista em pan e vetor de imagens
            parcial_color->clear(); imagens_baixa_resolucao.clear(); tilts_imagens_pan_atual.clear();
            // Vamos mudar de angulo em pan
            mudando_vista_pan = true;
            // Enviar para a proxima posicao
            if(indice_posicao + 1 < pans_raw.size()){
                indice_posicao++; // Proximo ponto de observacao
                cmd.request.pan_pos  = pans_raw[indice_posicao];
                cmd.request.tilt_pos = tilts_raw[indice_posicao];
                if(comando_motor.call(cmd))
                    ROS_INFO("Indo para a posicao %d de %zu totais aquisitar nova imagem ...", indice_posicao+1, pans_raw.size());
            } else { // Se for a ultima, finalizar
                indice_posicao++;
                ROS_INFO("Aquisitamos tudo, finalizando ...");
                cmd_led.request.led = 1; // LED continuo
                comando_leds.call(cmd_led);
                cmd.request.pan_pos  = pans_raw[ntilts*3]; // Quase no inicio, pra quando ligar dar uma mexida
                cmd.request.tilt_pos = raw_hor_tilt;
                comando_motor.call(cmd);
            }
            // Chaveando flag de processamento
            processar_nuvem_e_enviar = false;
        }

        // Controlando aqui o caminho dos servos, ate chegar ao final
        if(abs(pan - pans_raw[indice_posicao]) <= dentro && abs(tilt - tilts_raw[indice_posicao]) <= dentro && indice_posicao != pans_raw.size()){
            // Se chegamos na primeira captura, indice 0, podemos comecar a capturar o laser
            if(!iniciar_laser) iniciar_laser = true;
            ROS_INFO("Estamos captando a imagem %d ...", indice_posicao+1);
            // Libera captura da imagem
            aquisitar_imagem = true;
            for(int i=0; i<15; i++){
                r.sleep();
                ros::spinOnce();
            }
            // Chavear a flag
            aquisitar_imagem = false;
            // Se estavamos mudando de vista pan, nao estamos mais
            if(mudando_vista_pan)
                mudando_vista_pan = false;
            // Salvar a imagem na pasta certa
            string nome_imagem_atual;
            if(indice_posicao < 10){
                nome_imagem_atual = "imagem_00"+std::to_string(indice_posicao+1);
                pc->saveImage(image_ptr->image, nome_imagem_atual);
            } else if(indice_posicao+1 < 100) {
                nome_imagem_atual = "imagem_0"+std::to_string(indice_posicao+1);
                pc->saveImage(image_ptr->image, nome_imagem_atual);
            } else {
                nome_imagem_atual = "imagem_"+std::to_string(indice_posicao+1);
                pc->saveImage(image_ptr->image, nome_imagem_atual);
            }
            // Reduzir resolucao
            Mat im;
            image_ptr->image.copyTo(im);
            resize(im, im, Size(im.cols/4, im.rows/4));
            // Salvar imagem em baixa resolucao e odometria local
            imagens_baixa_resolucao.push_back(im);
            tilts_imagens_pan_atual.push_back(tilt);
            // Enviar pose da camera atual para o fog otimizar
            nav_msgs::Odometry odom_out;
            odom_out.pose.pose.position.x = pan;
            odom_out.pose.pose.position.y = tilts_imagens_pan_atual[tilts_imagens_pan_atual.size() - 1];
            odom_out.pose.pose.position.z = indice_posicao;
            odom_out.pose.pose.orientation.w = pans_raw.size(); // Quantidade total de aquisicoes para o fog ter nocao
            odom_out.header.stamp = ros::Time::now();
            odom_out.header.frame_id = "map";
            an_pub.publish(odom_out);
            // Se ainda nao inteiramos todos os niveis de tilt, avancar
            if(tilts_imagens_pan_atual.size() < ntilts){
                // Avancar uma posicao no vetor, se possivel
                dynamixel_workbench_msgs::JointCommand cmd;
                cmd.request.unit = "raw";
                if(indice_posicao + 1 < pans_raw.size()){
                    indice_posicao++; // Proximo ponto de observacao
                    cmd.request.pan_pos  = pans_raw[indice_posicao];
                    cmd.request.tilt_pos = tilts_raw[indice_posicao];
                    if(comando_motor.call(cmd))
                        ROS_INFO("Indo para a posicao %d de %zu totais aquisitar nova imagem ...", indice_posicao+1, pans_raw.size());
                }
            } else {
                // Se ja inteiramos, aguardar as aquisicoes, colorir a nuvem e enviar para fog
                // A outra rotina vai enviar para a proxima vista em pan para comecarmos novamente
                processar_nuvem_e_enviar = true;
            }
        }

        // Roda o loop de ROS
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

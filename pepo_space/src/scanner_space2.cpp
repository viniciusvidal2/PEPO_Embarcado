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
bool aquisitar_imagem = false, fim_aquisicao_imagens = false, mudando_vista_pan = false, pode_publicar = true;
// Publicador de imagem, nuvem parcial e odometria
ros::Publisher im_pub;
ros::Publisher cl_pub;
ros::Publisher od_pub;
// Classe de processamento de nuvens
ProcessCloud *pc;
// Controle de voltas realizadas no escaneamento do laser
int Nvoltas = 2, voltas_realizadas = 0;
// Nuvens de pontos e vetor de nuvens parciais
PointCloud<PointXYZ>::Ptr parcial;
PointCloud<PointXYZ>::Ptr parcial_enviar;
// Objetivo atual em tilt
int fim_curso_tilt;
// Valor do servo naquele instante em variavel global para ser acessado em varios callbacks
int pan, tilt, pan_enviar, tilt_enviar;

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
    if(aquisitar_imagem){
        // Capturar imagem
        cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
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
        // Publicar para o no em fog imagem e odometria
        cv_bridge::CvImage out_msg;
        out_msg.header   = msg->header;
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        out_msg.image    = im;
        nav_msgs::Odometry odom_out;
        odom_out.pose.pose.position.x = pan;
        odom_out.pose.pose.position.y = tilt;
        odom_out.header.stamp = out_msg.header.stamp;
        odom_out.header.frame_id = out_msg.header.frame_id;
        od_pub.publish(odom_out);
        im_pub.publish(out_msg.toImageMsg());
        // Avancar uma posicao no vetor, se possivel
        dynamixel_workbench_msgs::JointCommand cmd;
        cmd.request.unit = "raw";
        if(indice_posicao + 1 < pans_raw.size()){
            indice_posicao++; // Proximo ponto de observacao
            cmd.request.pan_pos  = pans_raw[indice_posicao];
            cmd.request.tilt_pos = tilts_raw[indice_posicao];
            if(comando_motor.call(cmd))
                ROS_INFO("Indo para a posicao %d de %zu totais aquisitar nova imagem ...", indice_posicao+1, pans_raw.size());
        } else { // Se for a ultima, finalizar
            ROS_INFO("Aquisitamos todas as imagens, comecando aquisicao do laser ...");
            fim_aquisicao_imagens = true;
        }
        // Chavear a flag
        aquisitar_imagem = false;
    }
}

/// Callback dos servos
///
void dynCallback(const nav_msgs::OdometryConstPtr& msg){
    // As mensagens trazem angulos em unidade RAW
    pan = int(msg->pose.pose.position.x), tilt = int(msg->pose.pose.position.y);
    if(!fim_aquisicao_imagens){
        // Se estiver perto do valor de posicao atual de gravacao, liberar a aquisicao
        if(abs(pan - pans_raw[indice_posicao]) <= dentro && abs(tilt - tilts_raw[indice_posicao]) <= dentro && !fim_aquisicao_imagens){
            sleep(2); // Espera servos pararem
            aquisitar_imagem = true;
            ROS_INFO("Estamos captando a imagem %d ...", indice_posicao+1);
        } else {
            aquisitar_imagem = false;
        }
    }
}

/// Callback do laser e servos
///
void laserServosCallback(const sensor_msgs::PointCloud2ConstPtr &msg_cloud, const nav_msgs::OdometryConstPtr &msg_servos){
    // As mensagens trazem angulos em unidade RAW
    pan = int(msg_servos->pose.pose.position.x), tilt = int(msg_servos->pose.pose.position.y);
    // Se nao estamos mudando vista pan, processar
    if(!mudando_vista_pan){
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
        /// Verificar a posicao que estamos e enviar para o oposto
        // Se ja demos o suficiente de voltas, nem entrar e verificar ali embaixo
        if(abs(tilt - fim_curso_tilt) <= dentro){
        if(voltas_realizadas < Nvoltas){
            // Atualizar voltas
            voltas_realizadas++;
            // Ajustar fim de curso para o tilt oposto
            fim_curso_tilt = (fim_curso_tilt == raw_max_tilt) ? raw_min_tilt : raw_max_tilt;
            // Enviar comando para a proxima posicao
            cmd.request.pan_pos  = pans_raw[indice_posicao];
            cmd.request.tilt_pos = fim_curso_tilt;
            if(comando_motor.call(cmd))
                ROS_INFO("Fazendo volta %d ...", voltas_realizadas);
        } else if(voltas_realizadas == Nvoltas){ // Se ja demos todas as voltas, enviar ao inicio e a proxima posicao de pan
            ROS_INFO("Salvando nuvem %d em PAN ...", indice_posicao+1);
            pode_publicar = false;
            // Preencher nuvem parcial atual para enviar junto com angulos de odometria
            VoxelGrid<PointXYZ> voxel;
            voxel.setLeafSize(0.02, 0.02, 0.02);
            voxel.setInputCloud(parcial);
            parcial_enviar->clear();
            voxel.filter(*parcial_enviar);
            pan_enviar = pan; tilt_enviar = tilt;            
            // Zerar voltas e atualizar indice de posicao
            voltas_realizadas = 0;
            indice_posicao++;
            pode_publicar = true;
            // Salvar nuvem
            if(indice_posicao+1 < 10)
                pc->saveCloud(parcial, "pf_00"+std::to_string(pans_deg.size()-indice_posicao+1));
            else if(indice_posicao+1 < 100)
                pc->saveCloud(parcial, "pf_0" +std::to_string(pans_deg.size()-indice_posicao+1));
            // Limpar nuvem parcial
            parcial->clear();
            // Se na ultima posicao de pan, finalizar
            if(indice_posicao == pans_deg.size()){
                cmd.request.pan_pos  = pans_raw[pans_raw.size()-1];
                cmd.request.tilt_pos = raw_hor_tilt;
                if(comando_motor.call(cmd))
                    ROS_INFO("Indo para baixo e finalizando tudo ...");
                // LED fixo
                led_control::LED cmd_led;
                cmd_led.request.led = 1;
                comando_leds.call(cmd_led);
                sleep(5);
                //system("gnome-terminal -x sh -c 'rosnode kill -a'");
                //ros::shutdown();
            } else { // Senao, mandar a proxima vista em pan
                // Chaveando flag para mudar a vista em pan
                mudando_vista_pan = true;
                // Indo para a proxima aquisicao
                cmd.request.pan_pos  = pans_raw[indice_posicao];
                cmd.request.tilt_pos = fim_curso_tilt;
                if(comando_motor.call(cmd))
                    ROS_INFO("Indo para posicao pan %d de %zu ...", indice_posicao+1, pans_raw.size());
            }
        }
        }
    } else { // Estamos mudando de vista pan, verificar se chegamos no inicio de nova aquisicao e chavear flag
        mudando_vista_pan = true;
        if(abs(pan - pans_raw[indice_posicao]) <= dentro)
            mudando_vista_pan = false;
    }
}

/// Main
///
int main(int argc, char **argv)
{
    ros::init(argc, argv, "scanner_space2");
    ros::NodeHandle nh;
    ros::NodeHandle n_("~");
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
    ROS_INFO("Iniciando o processo do SCANNER - aguardando servos ...");

    // Pegando o nome da pasta por parametro
    string nome_param;
    n_.param("pasta", nome_param, string("Dados_PEPO"));

    // Apagando pasta atual e recriando a mesma na area de trabalho
    char* home;
    home = getenv("HOME");
    string pasta = string(home)+"/Desktop/"+nome_param.c_str()+"/";
    system(("rm -r "+pasta).c_str());
    mkdir(pasta.c_str(), 0777);

    // Preenchendo vetor de pan e tilt primeiro para a camera
    int step = 30; // [DEG]
    // Pontos de observacao em tilt
    vector<float> tilts_camera_deg {deg_min_tilt, deg_hor_tilt, -30.0f, deg_max_tilt};
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

    // Enviando scanner para o inicio
    cmd.request.unit = "raw";
    cmd.request.pan_pos  = pans_raw[0];
    cmd.request.tilt_pos = tilts_raw[0];

    ros::Rate rd(1);
    while(!comando_motor.call(cmd)){
        ROS_ERROR("Esperando a comunicacao com os servos !! ...");
        ros::spinOnce();
        rd.sleep();
    }
    ROS_INFO("Servos comunicando e indo para a posicao inicial ...");
    sleep(2); // Esperar os servos pararem de balancar

    // Inicia classe de processo de nuvens
    pc = new ProcessCloud(pasta);

    // Publicadores
    im_pub = nh.advertise<sensor_msgs::Image      >("/image_space", 10);
    cl_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_space", 10);
    od_pub = nh.advertise<nav_msgs::Odometry      >("/angle_space", 10);

    // Subscribers dessincronizados para mensagens de imagem e motores
    ros::Subscriber sub_cam = nh.subscribe("/camera/image_raw"               , 10, camCallback);
    ros::Subscriber sub_dyn = nh.subscribe("/dynamixel_angulos_sincronizados", 10, dynCallback);

    ROS_INFO("Comecando a aquisicao ...");

    // LED piscando lentamente - AQUISITANDO
    cmd_led.request.led = 3;
    comando_leds.call(cmd_led);

    // Aguardando a aquisicao das imagens acabar, para depois pegar tudo com o laser
    ros::Rate r(10);
    while(!fim_aquisicao_imagens){
        r.sleep();
        ros::spinOnce();
    }
    // Dar um tempo pra receber a ultima imagem no outro subscriber
    sleep(5);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// Daqui pra frente colocaremos tudo do laser
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Apagar o subscriber da camera
    sub_cam.shutdown(); im_pub.shutdown(); sub_dyn.shutdown();
    // Limpar vetores de posicao e posicao atual
    pans_raw.clear(); pans_deg.clear();
    tilts_raw.clear(); tilts_deg.clear();
    indice_posicao = 0;

    // Controlar somente aqui as posicoes em pan, em tilt ficar variando de acordo com as voltas
    pans_deg = pans_camera_deg;
    for(int i=pans_deg.size()-1; i>=0 ; i--)
        pans_raw.push_back(deg2raw(pans_deg[i], "pan"));

    // Iniciando a nuvem parcial acumulada de cada pan
    parcial = (PointCloud<PointXYZ>::Ptr) new PointCloud<PointXYZ>();
    parcial->header.frame_id  = "map";
    parcial_enviar = (PointCloud<PointXYZ>::Ptr) new PointCloud<PointXYZ>();
    parcial_enviar->header.frame_id  = "map";

    // Iniciar subscritor de laser sincronizado com posicao dos servos
    ros::NodeHandle nh2;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_nuvem (nh2, "/livox/lidar"                    , 10);
    message_filters::Subscriber<nav_msgs::Odometry      > sub_odom  (nh2, "/dynamixel_angulos_sincronizados", 10);
    Synchronizer<syncPolicy> sync(syncPolicy(10), sub_nuvem, sub_odom);
    message_filters::Connection conexao = sync.registerCallback(boost::bind(&laserServosCallback, _1, _2));

    // Reforcar ida do servo a ultima posicao de aquisicao
    ROS_INFO("Comecando a aquisicao do laser ...");
    cmd.request.pan_pos  = pans_raw[0];
    cmd.request.tilt_pos = raw_max_tilt;
    fim_curso_tilt = raw_max_tilt; // Acerta o fim de curso atual
    comando_motor.call(cmd);

    // Aguardar todo o processamento e ir publicando
    while(ros::ok()){
        // Publicar nuvem
        if(parcial_enviar->size() > 10 && pode_publicar){
            sensor_msgs::PointCloud2 msg_out;
            toROSMsg(*parcial_enviar, msg_out);
            msg_out.header.stamp = ros::Time::now();
            msg_out.header.frame_id = "map";
            nav_msgs::Odometry odom_out;
            odom_out.pose.pose.position.x = pan_enviar;
            odom_out.pose.pose.position.y = tilt_enviar;
            odom_out.pose.pose.position.z = indice_posicao;
            odom_out.header.stamp = msg_out.header.stamp;
            odom_out.header.frame_id = msg_out.header.frame_id;
            od_pub.publish(odom_out);
            cl_pub.publish(msg_out);
        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

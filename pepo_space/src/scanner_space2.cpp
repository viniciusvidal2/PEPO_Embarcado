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

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <dynamixel_workbench_msgs/DynamixelCommand.h>
#include <dynamixel_workbench_msgs/DynamixelInfo.h>
#include <dynamixel_workbench_msgs/JointCommand.h>
#include <dynamixel_workbench_msgs/DynamixelState.h>

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
int dentro = 4; // [RAW]
// Flags de controle
bool aquisitar_imagem = false, fim_aquisicao_imagens = false, mudando_vista_pan = false;
// Publicador de imagem, nuvem parcial e odometria
ros::Publisher im_pub;
ros::Publisher cl_pub;
ros::Publisher od_pub;
// Classe de processamento de nuvens
ProcessCloud *pc;
// Controle de voltas realizadas no escaneamento do laser
int Nvoltas = 10, voltas_realizadas = 0;
// Nuvens de pontos e vetor de nuvens parciais
PointCloud<PointT>::Ptr parcial;

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
        resize(im, im, Size(im.cols/2, im.rows/2));
        // Publicar para o no em fog
        cv_bridge::CvImage out_msg;
        out_msg.header   = msg->header;
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        out_msg.image    = im;
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
            // Voltando para o inicio
            cmd.request.pan_pos  = pans_raw[0];
            cmd.request.tilt_pos = tilts_raw[0]; // Vai deitar mesmo
            comando_motor.call(cmd);
            ROS_INFO("Aquisitamos todas as imagens, indo para a posicao inicial ...");
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
    int pan = int(msg->pose.pose.position.x), tilt = int(msg->pose.pose.position.y);
    // Se estiver perto do valor de posicao atual de gravacao, liberar a aquisicao
    if(abs(pan - pans_raw[indice_posicao]) <= dentro && abs(tilt - tilts_raw[indice_posicao]) <= dentro){
        sleep(1); // Espera servos pararem
        aquisitar_imagem = true;
        ROS_INFO("Estamos captando a imagem %d ...", indice_posicao+1);
    } else {
        aquisitar_imagem = false;
    }
}

/// Callback sincronizado de servos e laser
///
void callback_cloud_servos(const sensor_msgs::PointCloud2ConstPtr &msg_cloud, const nav_msgs::OdometryConstPtr &msg_motor){
    int pan = msg_motor->pose.pose.position.x, tilt = msg_motor->pose.pose.position.y;
    // Se nao estamos mudando vista pan, processar
    if(!mudando_vista_pan){
        // Converter mensagem
        PointCloud<PointT>::Ptr cloud (new PointCloud<PointT>);
        fromROSMsg(*msg_cloud, *cloud);
        // Transformando para o frame da camera
        pc->transformToCameraFrame(cloud);
        // Aplicar transformada de acordo com o angulo - somente tilt
        float t = raw2deg(msg_motor->pose.pose.position.y, "tilt");
        Matrix3f R = pc->euler2matrix(0, DEG2RAD(-t), 0);
        Matrix4f T;
        T.block<3,3>(0, 0) = R;
        transformPointCloud(*cloud, *cloud, T);
        // Acumular nuvem
        *parcial += *cloud;
        /// Verificar a posicao que estamos e enviar para a proxima parada
        // Se estamos no minimo de tilt, enviar para o maximo e atualizar voltas
        // Se ja demos o suficiente de voltas, nem entrar e verificar ali embaixo
        if(abs(tilt - raw_min_tilt) <= dentro && voltas_realizadas < Nvoltas){
            // Atualizar voltas
            voltas_realizadas++;
            // Enviar comando para a proxima posicao
            cmd.request.pan_pos  = pans_raw[indice_posicao];
            cmd.request.tilt_pos = raw_max_tilt;
            if(comando_motor.call(cmd))
                ROS_INFO("Subindo apos %d voltas ...", voltas_realizadas-1);
        }
        // Se estamos no maximo de tilt, enviar para o minimo e atualizar voltas
        // Se ja demos o suficiente de voltas, nem entrar e verificar ali embaixo
        if(abs(tilt - raw_max_tilt) <= dentro && voltas_realizadas < Nvoltas){
            // Atualizar voltas
            voltas_realizadas++;
            // Enviar comando para a proxima posicao
            cmd.request.pan_pos  = pans_raw[indice_posicao];
            cmd.request.tilt_pos = raw_min_tilt;
            if(comando_motor.call(cmd))
                ROS_INFO("Descendo apos %d voltas ...", voltas_realizadas-1);
        }
        // Se ja demos todas as voltas, enviar ao inicio e a proxima posicao de pan
        if(voltas_realizadas == Nvoltas){
            // Zerar voltas e atualizar indice de posicao
            voltas_realizadas = 0;
            indice_posicao++;
            // Salvar nuvem
            if(indice_posicao+1 < 10)
                pc->saveCloud(cloud, "pf_00"+std::to_string(indice_posicao));
            else if(indice_posicao+1 < 100)
                pc->saveCloud(cloud, "pf_0"+std::to_string(indice_posicao));
            // Publicar nuvem
            sensor_msgs::PointCloud2 msg_out;
            toROSMsg(*cloud, msg_out);
            msg_out.header.stamp = ros::Time::now();
            cl_pub.publish(msg_out);
            // Publicar odometria
            nav_msgs::Odometry odom_out;
            odom_out = *msg_motor;
            odom_out.header.stamp = msg_out.header.stamp;
            odom_out.header.frame_id = msg_out.header.frame_id;
            od_pub.publish(odom_out);
            // Limpar nuvem parcial
            parcial->clear();
            // Se na ultima posicao de pan, finalizar
            if(indice_posicao == pans_raw.size()){
                cmd.request.pan_pos  = pans_raw[0];
                cmd.request.tilt_pos = raw_hor_tilt;
                if(comando_motor.call(cmd))
                    ROS_INFO("Indo para baixo e finalizando tudo ...");
                sleep(5);
                system("gnome-terminal -x sh -c 'rosnode kill -a'");
                ros::shutdown();
            } else { // Senao, mandar a proxima vista em pan
                // Chaveando flag para mudar a vista em pan
                mudando_vista_pan = true;
                // Indo para a proxima aquisicao
                cmd.request.pan_pos  = pans_raw[indice_posicao];
                cmd.request.tilt_pos = tilt;
                if(comando_motor.call(cmd))
                    ROS_INFO("Indo para posicao pan %d de %zu ...", indice_posicao, pans_raw.size());
            }
        }
    } else { // Estamos mudando de vista pan, verificar se chegamos no inicio de nova aquisicao e chavear flag
        if((abs(tilt - raw_max_tilt) <= dentro || abs(tilt - raw_min_tilt) <= dentro) && abs(pan - pans_raw[indice_posicao]) <= dentro)
            mudando_vista_pan = false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scanner_space2");
    ros::NodeHandle nh;
    ros::NodeHandle n_("~");
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
    // Enchendo vetores de comandos em deg globais
    for(int j=0; j < tilts_camera_deg.size(); j++){
        for(int i=0; i < pans_camera_deg.size(); i++){
            if(remainder(j, 2) == 0){
                pans_deg.push_back(pans_camera_deg[i]);
                pans_raw.push_back(deg2raw(pans_camera_deg[i], "pan"));
            } else {
                pans_deg.push_back(pans_camera_deg[pans_camera_deg.size() - 1 - i]);
                pans_raw.push_back(deg2raw(pans_camera_deg[pans_camera_deg.size() - 1 - i], "pan"));
            }
            tilts_deg.push_back(tilts_camera_deg[j]);
            tilts_raw.push_back(deg2raw(tilts_camera_deg[j], "tilt"));
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
    ros::Subscriber sub_cam   = nh.subscribe("/camera/image_raw"               , 10, camCallback  );
    ros::Subscriber sub_dyn   = nh.subscribe("/dynamixel_angulos_sincronizados", 10, dynCallback  );

    ROS_INFO("Comecando a aquisicao ...");

    // LED piscando lentamente - AQUISITANDO
    cmd_led.request.led = 3;
    comando_leds.call(cmd_led);

    // Aguardando a aquisicao das imagens acabar, para depois pegar tudo com o laser
    ros::Rate r(1);
    while(!fim_aquisicao_imagens){
        r.sleep();
        ros::spinOnce();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// Daqui pra frente colocaremos tudo do laser
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Apagar os subscribers anteriores
    sub_cam.shutdown();
    // Limpar vetores de posicao e posicao atual
    pans_raw.clear(); pans_deg.clear();
    tilts_raw.clear(); tilts_deg.clear();
    indice_posicao = 0;

    // Controlar somente aqui as posicoes em pan, em tilt ficar variando de acordo com as voltas
    pans_deg = pans_camera_deg;

    // Iniciando a nuvem parcial acumulada de cada pan
    parcial = (PointCloud<PointT>::Ptr) new PointCloud<PointT>();
    parcial->header.frame_id  = "pepo";

    // Iniciar subscritor de laser sincronizado com posicao dos servos
    message_filters::Subscriber<sensor_msgs::PointCloud2> laser_sub(nh, "/livox/lidar"                    , 10);
    message_filters::Subscriber<nav_msgs::Odometry      > motor_sub(nh, "/dynamixel_angulos_sincronizados", 10);
    Synchronizer<syncPolicy> Sync(syncPolicy(10), laser_sub, motor_sub);
    Sync.registerCallback(boost::bind(&callback_cloud_servos, _1, _2));

    // Aguardar todo o processamento
    ros::spin();

    return 0;
}

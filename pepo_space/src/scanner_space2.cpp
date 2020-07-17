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

#include "../../libraries/include/processcloud.h"
#include "led_control/LED.h"

/// Namespaces
///
using namespace pcl;
using namespace pcl::io;
using namespace cv;
using namespace std;

/// Definiçoes
///
typedef PointXYZRGB PointT;

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
// Servico para controlar o LED
ros::ServiceClient comando_leds;
// Posicao atual de aquisicao
int indice_posicao = 0;
// Raio de aceitacao de posicao angular
int dentro = 4; // [RAW]
// Flags de controle
bool aquisitar_imagem = false, fim_aquisicao_imagens = false;;
// Publicador de imagem
ros::Publisher im_pub;
// Classe de processamento de nuvens
ProcessCloud *pc;

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
            pans_deg[i + j*pans_camera_deg.size()]  = pans_camera_deg[i];
            tilts_deg[i + j*pans_camera_deg.size()] = tilts_camera_deg[j];
            pans_raw.push_back(deg2raw(pans_camera_deg[i], "pan"));
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
    dynamixel_workbench_msgs::JointCommand cmd;
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
    im_pub = nh.advertise<sensor_msgs::Image>("/image_space", 10);

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

    // Daqui pra frente colocaremos tudo do laser
    return 0;


}

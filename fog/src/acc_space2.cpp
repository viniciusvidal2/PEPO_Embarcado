/// Includes
///
#include <iostream>
#include <ros/ros.h>
#include <string>
#include <math.h>

#include <std_msgs/Float32.h>
#include <sensor_msgs/NavSatFix.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "../../libraries/include/processcloud.h"
#include "../../libraries/include/dynamixelservos.h"

/// Namespaces
///
using namespace pcl;
using namespace pcl::io;
using namespace cv;
using namespace std;
using namespace message_filters;

/// Definiçoes
///
typedef PointXYZRGB       PointT ;
typedef PointXYZRGBNormal PointTN;
typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;

/// Variaveis globais
///
// Publicador de feedback
ros::Publisher msg_pub;
// Classe de processamento de nuvens e propriedades dos servos
DynamixelServos *ds;
ProcessCloud *pc;
// Nuvens de pontos acumulada e anterior
PointCloud<PointT>::Ptr acc;
// Nome da pasta que vamos trabalhar
string pasta;
// Vetor com linhas do arquivo NVM
vector<string> linhas_sfm;
// Quantos tilts estao ocorrendo por pan, e contador de quantos ja ocorreram
int contador_nuvens = 0;
// Pontos de vista para nao salvar nuvens (olhando para o chao)
vector<int> nao_salvar_nuvens{1, 16, 17, 32, 33, 48, 49, 64, 65, 80, 81, 96, 97, 112, 113};
// Coordenadas de GPS medias locais
float lat_avg, lon_avg, alt_avg;

///////////////////////////////////////////////////////////////////////////////////////////
string find_current_folder(string p){
    struct stat buffer;
    string ultimo_nome = p + std::to_string(0);
    for(int i=1; i<200; i++){ // Tentar criar ate 200 pastas - impossivel
        string nome_atual = p + std::to_string(i);
        if(stat(nome_atual.c_str(), &buffer)) // Se nao existe a pasta
            return ultimo_nome;

        ultimo_nome = nome_atual;
    }
}
void saveTempCloud(PointCloud<PointT>::Ptr cloud, int n){
    bool salvar = true;
    for(auto id:nao_salvar_nuvens){
        if(n == id) salvar = false;
    }
    // Salva a parcial daquela vista
    if(salvar){
        if(n < 10){
            pc->saveCloud(cloud, "c_00"+std::to_string(n));
        } else if(n < 100) {
            pc->saveCloud(cloud, "c_0" +std::to_string(n));
        } else {
            pc->saveCloud(cloud, "c_"  +std::to_string(n));
        }
    }
}
void writeGPSdata(){
    // Gravar um arquivo de coordenadas na pasta do scan
    string nome_arquivo = pasta + "gps.txt";
    ofstream gps(nome_arquivo);
    if(gps.is_open()){

        gps << std::to_string(lat_avg) + "\n";
        gps << std::to_string(lon_avg) + "\n";
        gps << std::to_string(alt_avg) + "\n";

    }
    gps.close(); // Fechar para nao ter erro
}
///////////////////////////////////////////////////////////////////////////////////////////

/// Callback do GPS
///
void gpsCallback(const sensor_msgs::NavSatFixConstPtr &msg){
    // Adicionar leitura de forma ponderada com um filtro passa baixa
    lat_avg = 0.7*lat_avg + 0.3*msg->latitude;
    lon_avg = 0.7*lon_avg + 0.3*msg->longitude;
    alt_avg = 0.7*alt_avg + 0.3*msg->altitude;
}

/// Callback do laser e odometria sincronizado
///
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg_cloud, const nav_msgs::OdometryConstPtr& msg_angle){
    // Contador de qual nuvem estamos
    int cont_aquisicao = msg_angle->pose.pose.orientation.y + 1;

    // Converter mensagem em nuvem
    PointCloud<PointT>::Ptr cloud (new PointCloud<PointT>);
    fromROSMsg(*msg_cloud, *cloud);

    // Retirar pontos que nao foram coloridos pela imagem
    pc->cleanNotColoredPoints(cloud);

    // Salva as nuvens temporarias
    saveTempCloud(cloud, cont_aquisicao);

    *acc += *cloud;

    // Fazendo processos finais
    if(cont_aquisicao >= msg_angle->pose.pose.orientation.w){
        ROS_INFO("Salvando coordenadas GPS ...");
        writeGPSdata();

        ROS_INFO("Salvando blueprint ...");
        Mat blueprint;
        float side_area = 30, side_res = 0.04;
        pc->blueprint(acc, side_area, side_res, blueprint);
        acc->clear();

        ros::Duration(3).sleep();
        std_msgs::Float32 msg_feedback;
        msg_feedback.data = 100.0;
        msg_pub.publish(msg_feedback);
        sleep(3);
        msg_feedback.data = 1.0;
        msg_pub.publish(msg_feedback);

        // Esse no so finaliza a si mesmo, os outros sao finalizados no edge
        ROS_INFO("Processo finalizado.");
        ros::shutdown();
    }
    ////////////////
    // Fala a porcentagem do total que ja resolvemos
    std_msgs::Float32 msg_feedback;
    msg_feedback.data = 100.0*float(cont_aquisicao)/float(msg_angle->pose.pose.orientation.w);
    if(msg_feedback.data < 100.0)
        msg_pub.publish(msg_feedback);
}

/// Main
///
int main(int argc, char **argv)
{
    ros::init(argc, argv, "acc_space2");
    ros::NodeHandle nh;
    ros::NodeHandle n_("~");
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
    ROS_INFO("Iniciando o processo em FOG de registro de nuvens ...");

    // Tempo para a pasta ser criada corretamente pelo outro no de contato com os sensores em edge
    sleep(5);

    // Classe de propriedades dos servos - no inicio para ja termos em maos
    ds = new DynamixelServos();

    // Pegando o nome da pasta por parametro
    string nome_param;
    n_.param<string>("pasta", nome_param , string("Dados_PEPO"));

    char* home;
    home = getenv("HOME");
    // Checando ultima pasta criada, pegar ela
    pasta = string(home)+"/Desktop/ambientes/"+nome_param;
    pasta = find_current_folder(pasta + "/scan") + "/";

    // Inicia classe de processo de nuvens
    pc  = new ProcessCloud(pasta);

    // Iniciando a nuvem parcial acumulada de cada pan
    acc = (PointCloud<PointT>::Ptr) new PointCloud<PointT>();
    acc->header.frame_id = "map";

    // Publicador de quanto esta sendo
    msg_pub = nh.advertise<std_msgs::Float32>("/feedback_scan", 10);
    std_msgs::Float32 msg_feedback;
    msg_feedback.data = 1.0; // Para a camera liberar no aplicativo
    ros::Rate r(2);
    for(int i=0; i<5; i++){
        msg_pub.publish(msg_feedback);
        r.sleep();
    }

    // Iniciar subscritor da nuvem sincronizado com odometria
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, "/cloud_space", 100);
    message_filters::Subscriber<nav_msgs::Odometry      > angle_sub(nh, "/angle_space", 100);
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), cloud_sub, angle_sub);
    sync.registerCallback(boost::bind(&cloudCallback, _1, _2));

    // Iniciar subscritor do GPS
    ros::Subscriber gps_sub = nh.subscribe("/gps", 10, gpsCallback);

    ROS_INFO("Comecando a reconstrucao do space ...");

    // Aguardar todo o processamento e ir publicando
    ros::spin();

    return 0;
}

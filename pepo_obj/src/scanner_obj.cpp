/// Includes
///
#include <iostream>
#include <ros/ros.h>
#include <ros/service_server.h>
#include <string>
#include <math.h>

#include <pcl/filters/passthrough.h>

#include <std_msgs/Float32.h>

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
ProcessImages *pi;
// Contador de aquisicoes - usa tambem para salvar no lugar certo
int cont_aquisicao = 0;
// Publisher para feedback
ros::Publisher feedback_pub;

string pasta;

string create_folder(string p){
    struct stat buffer;
    for(int i=1; i<200; i++){ // Tentar criar ate 200 pastas - impossivel
        string nome_atual = p + std::to_string(i);
        if(stat(nome_atual.c_str(), &buffer)){ // Se nao existe a pasta
            mkdir(nome_atual.c_str(), 0777);
            return nome_atual;
        }
    }
}

/// Callback da camera
///
void camCallback(const sensor_msgs::ImageConstPtr& msg){
    // Aqui ja temos a imagem em ponteiro de opencv, depois de pegar uma desabilitar
    if(aquisitar_imagem)
        image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
}

/// Servico para controle de aquisicao
///
bool capturar_obj(pepo_obj::comandoObj::Request &req, pepo_obj::comandoObj::Response &res){
    if(req.comando == 1){ // Havera mais uma nova aquisicao
        aquisitando = true;
        aquisitar_imagem = true;
        res.result = 1;
        ROS_INFO("Realizando aquisicao na posicao %d ...", cont_aquisicao+1);
        ros::Rate r2(1);
        int tempo_total = 5; // [s]
        std_msgs::Float32 msg_feedback;

        for(int i=0; i<tempo_total; i++){
            r2.sleep();
            ros::spinOnce();
            msg_feedback.data = (100*float(i)/float(tempo_total) > 0) ? 100*float(i)/float(tempo_total) : 1 ;
            feedback_pub.publish(msg_feedback);
        }

        aquisitar_imagem = false;
        string nome_imagem_atual;
        if(cont_aquisicao + 1 < 10)
            nome_imagem_atual = "imagem_00"+std::to_string(cont_aquisicao+1);
        else if(cont_aquisicao+1 < 100)
            nome_imagem_atual = "imagem_0" +std::to_string(cont_aquisicao+1);
        else
            nome_imagem_atual = "imagem_"  +std::to_string(cont_aquisicao+1);
        pi->saveImage(image_ptr->image, nome_imagem_atual);

        cont_aquisicao++;

        sleep(5);
        msg_feedback.data = 100;
        feedback_pub.publish(msg_feedback);
        sleep(3);
        msg_feedback.data = 1;
        feedback_pub.publish(msg_feedback);

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
    // Checando se ha a pasta spaces, senao criar
    pasta = string(home)+"/Desktop/objetos/";
    struct stat buffer;
    if(stat(pasta.c_str(), &buffer)) // Se nao existe a pasta
        mkdir(pasta.c_str(), 0777);
    // Criando pasta mae
    pasta = pasta + nome_param.c_str();
    if(stat(pasta.c_str(), &buffer)) // Se nao existe a pasta
        mkdir(pasta.c_str(), 0777);
    // Criando pastas filhas
    pasta = create_folder(pasta + "/scan") + "/";

    // Inicia classe de processo de nuvens
    pi = new ProcessImages(pasta);

    // Publisher do feedback pra mostrar que tem procedimento
    feedback_pub = nh.advertise<std_msgs::Float32>("/feedback_scan", 10);
    std_msgs::Float32 msg;
    msg.data = 1;
    ros::Rate r5(2);
    for(int i=0; i<5; i++){
        feedback_pub.publish(msg);
        r5.sleep();
    }

    // Inicia servidor que recebe o comando sobre como proceder com a aquisicao
    ros::ServiceServer procedimento = nh.advertiseService("/capturar_obj", capturar_obj);

    // Subscribers dessincronizados para mensagens de laser, imagem e motores
    ros::Subscriber sub_cam = nh.subscribe("/camera/image_raw", 10, camCallback  );

    ROS_INFO("Comecando a aquisicao ...");

    ros::Rate r(10);
    while(ros::ok()){
        r.sleep();
        ros::spinOnce();

        if(fim_processo){
            system("rosnode kill camera imagem_lr_app scanner_obj");
            ros::shutdown();
            break;
        }
    }

    return 0;
}

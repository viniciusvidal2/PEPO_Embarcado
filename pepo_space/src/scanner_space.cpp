/// Includes
///
#include <iostream>
#include <ros/ros.h>
#include <string>
#include <math.h>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

#include <dynamixel_workbench_msgs/DynamixelCommand.h>
#include <dynamixel_workbench_msgs/DynamixelInfo.h>
#include <dynamixel_workbench_msgs/JointCommand.h>
#include <dynamixel_workbench_msgs/DynamixelState.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "../../libraries/include/processcloud.h"
#include "../../libraries/include/processimages.h"
#include "led_control/LED.h"

/// Namespaces
///
using namespace cv;
using namespace std;

/// Variaveis globais
///
// Pasta onde vao ser salvos os arquivos
string pasta;
// Posicao atual de aquisicao
int indice_posicao = 0;
// Ponteiro de cv_bridge para a imagem
cv_bridge::CvImagePtr image_ptr;
ProcessImages *pi;
bool aquisitar_imagem = false;

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

/// Main
///
int main(int argc, char **argv)
{
    ros::init(argc, argv, "scanner_space");
    ros::NodeHandle nh;
    ros::NodeHandle n_("~");
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
    ROS_INFO("Iniciando o processo do SCANNER - aguardando servos ...");

    // Pegando os parametros
    string nome_param;
    int qualidade; // Qualidade a partir de quanto tempo vamos parar em uma vista aquisitando laser e imagem
    n_.param<string>("pasta", nome_param, string("Dados_PEPO"));

    char* home;
    home = getenv("HOME");
    // Checando se ha a pasta spaces, senao criar
    pasta = string(home)+"/Desktop/ambientes/";
    struct stat buffer;
    if(stat(pasta.c_str(), &buffer)) // Se nao existe a pasta
        mkdir(pasta.c_str(), 0777);
    // Criando pasta mae
    pasta = pasta + nome_param.c_str();
    if(stat(pasta.c_str(), &buffer)) // Se nao existe a pasta
        mkdir(pasta.c_str(), 0777);
    // Criando pastas filhas
    pasta = create_folder(pasta + "/scan") + "/";

    // Iniciando ponteiro de imagem em cv_bridge
    image_ptr = (cv_bridge::CvImagePtr) new cv_bridge::CvImage;
    pi = new ProcessImages(pasta);

    // Subscribers
    ros::Subscriber sub_cam = nh.subscribe("/camera/image_raw", 10, camCallback);

    // Publicadores
    ros::Publisher msg_pub = nh.advertise<std_msgs::Float32>("/feedback_scan", 10);
    std_msgs::Float32 msg;
    msg.data = 0;
    msg_pub.publish(msg);

    ros::Rate r(20), r2(1), r3(0.2);
    int maximo = 20, contador;
    while(ros::ok()){

	contador = 0;
        while(ros::ok()){
            contador++;
            r2.sleep();
            ros::spinOnce();
            if(contador == 10) break;
        }
        aquisitar_imagem = true;
        for(int i=0; i<5; i++){
            r2.sleep();
            ros::spinOnce();
        }
        aquisitar_imagem = false;
        Mat imagem_temp;
        image_ptr->image.copyTo(imagem_temp);
        // Salvar a imagem na pasta certa
        string nome_imagem_atual;
        if(indice_posicao + 1 < 10)
            nome_imagem_atual = "imagem_00"+std::to_string(indice_posicao+1);
        else if(indice_posicao+1 < 100)
            nome_imagem_atual = "imagem_0" +std::to_string(indice_posicao+1);
        else
            nome_imagem_atual = "imagem_"  +std::to_string(indice_posicao+1);
        pi->saveImage(imagem_temp, nome_imagem_atual);
        msg.data = 100.0*float(indice_posicao+1)/float(maximo);
        msg_pub.publish(msg);
        indice_posicao++;

        // Roda o loop de ROS
        ros::spinOnce();
        r.sleep();

        if(indice_posicao == maximo){
            msg.data = 100;
            msg_pub.publish(msg);
            system("rosnode kill camera scanner_space");
            ros::shutdown();
            break;
        }

    }

    return 0;
}

/// Includes
///
#include <iostream>
#include <ros/ros.h>
#include <ros/service_server.h>
#include <string>
#include <math.h>

#include <std_msgs/Float32.h>

#include <pcl/filters/passthrough.h>

#include "../../libraries/include/processcloud.h"
#include "../../libraries/include/processimages.h"
#include "pepo_obj/comandoObj.h"

#include "led_control/LED.h"
#include "communication/state.h"

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
// Imagem com menor blur, para a maior covariancia encontrada no escaneamento
Mat min_blur_im, lap, lap_gray;
float max_var = 0;
bool aquisitando = false, aquisitar_imagem = false, fim_processo = false;
int contador_nuvem = 0, N = 200; // Quantas nuvens aquisitar em cada parcial
// Classe de processamento de nuvens
ProcessCloud *pc;
ProcessImages *pi;
// Nuvem de pontos parciais
PointCloud<PointXYZ>::Ptr parcial;
// Testando sincronizar subscribers por mutex
Mutex m;
// Contador de aquisicoes - usa tambem para salvar no lugar certo
int cont_aquisicao = 0;
// Publisher para feedback
ros::Publisher feedback_pub;

///////////////////////////////////////////////////////////////////////////////////////////
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
///////////////////////////////////////////////////////////////////////////////////////////

/// Callback da camera
///
void camCallback(const sensor_msgs::ImageConstPtr& msg){
    // Aqui ja temos a imagem em ponteiro de opencv, depois de pegar uma desabilitar
    if(aquisitar_imagem){
        image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        if(min_blur_im.cols < 10)
            image_ptr->image.copyTo(min_blur_im);
        // Converte escala de cinza
        cvtColor(image_ptr->image, lap_gray, COLOR_BGR2GRAY);
        // Variancia
        Laplacian(lap_gray, lap, CV_16SC1, 3, 1, 0, cv::BORDER_DEFAULT);
        Scalar m, s;
        meanStdDev(lap, m, s, Mat());
        // Checar contra maior variancia
        if(s[0] > max_var){
            image_ptr->image.copyTo(min_blur_im);
            max_var = s[0];
        }
    }
}

/// Callback do laser
///
void laserCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
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
            pass.setFilterLimits(0, 60); // Z metros de profundidade
            pass.filter(*cloud_color);
            // Colorir pontos com calibracao default para visualizacao rapida
            ROS_WARN("Colorindo nuvem para salvar com parametros default ...");
            pc->colorCloudWithCalibratedImage(cloud_color, image_ptr->image, 1);
            // Salvar dados parciais na pasta no Desktop
            ROS_WARN("Salvando dados de imagem e nuvem da aquisicao %d ...", cont_aquisicao);
            if(cont_aquisicao < 10){
                pi->saveImage(min_blur_im, "imagem_00"+std::to_string(cont_aquisicao));
                pc->saveCloud(cloud_color, "pf_00"+std::to_string(cont_aquisicao));
            } else if(cont_aquisicao < 100) {
                pi->saveImage(min_blur_im, "imagem_0"+std::to_string(cont_aquisicao));
                pc->saveCloud(cloud_color, "pf_0"+std::to_string(cont_aquisicao));
            } else {
                pi->saveImage(min_blur_im, "imagem_"+std::to_string(cont_aquisicao));
                pc->saveCloud(cloud_color, "pf_"+std::to_string(cont_aquisicao));
            }
            //////////////////////
            // Zerar contador de nuvens da parcial e anunciar 100%
            contador_nuvem = 0;
            min_blur_im.release(); max_var = 0; // Liberando a imagem para a proxima captura
            ROS_WARN("Terminada aquisicao da nuvem %d", cont_aquisicao);

        } else {
            contador_nuvem++;
        }

        // Falar a porcentagem da aquisicao para o usuario
        if(contador_nuvem % 40 == 0){
            std_msgs::Float32 msg_feedback;
            msg_feedback.data = (100.0 * float(contador_nuvem)/float(N) > 1) ? 100.0 * float(contador_nuvem)/float(N) : 1;
            feedback_pub.publish(msg_feedback);
        }
    }
}

/// Servico para controle de aquisicao
///
bool capturar_obj(pepo_obj::comandoObj::Request &req, pepo_obj::comandoObj::Response &res){
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
    string pasta = string(home)+"/Desktop/objetos/";
    struct stat buffer;
    if(stat(pasta.c_str(), &buffer)) // Se nao existe a pasta
        mkdir(pasta.c_str(), 0777);
    // Criando pasta mae
    pasta = pasta + nome_param.c_str();
    if(stat(pasta.c_str(), &buffer)) // Se nao existe a pasta
        mkdir(pasta.c_str(), 0777);
    // Criando pastas filhas
    pasta = create_folder(pasta + "/scan") + "/";

    // Inicia nuvem parcial acumulada a cada passagem do laser
    parcial = (PointCloud<PointXYZ>::Ptr) new PointCloud<PointXYZ>();
    parcial->header.frame_id  = "pepo";

    // Inicia classe de processo de nuvens
    pc = new ProcessCloud(pasta);
    pi = new ProcessImages(pasta);

    // Inicia servidor que recebe o comando sobre como proceder com a aquisicao
    ros::ServiceServer procedimento = nh.advertiseService("/capturar_obj", capturar_obj);

    // Publicadores
    feedback_pub = nh.advertise<std_msgs::Float32>("/feedback_scan", 10);

    // Subscribers dessincronizados para mensagens de laser e imagem
    ros::Subscriber sub_laser = nh.subscribe("/livox/lidar"     , 10, laserCallback);
    ros::Subscriber sub_cam   = nh.subscribe("/camera/image_raw", 10, camCallback  );

    ROS_INFO("Comecando a aquisicao ...");
    std_msgs::Float32 msg_feedback;
    msg_feedback.data = 1.0; // Para a camera liberar no aplicativo
    ros::Rate r(2);
    for(int i=0; i<5; i++){
        feedback_pub.publish(msg_feedback);
        r.sleep();
    }
    while(ros::ok()){
        r.sleep();
        ros::spinOnce();

        if(fim_processo){
            system("rosnode kill camera livox_lidar_publisher imagem_lr_app scanner_obj");
            ros::shutdown();
            break;
        }
    }

    return 0;
}

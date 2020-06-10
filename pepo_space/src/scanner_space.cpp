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

/// Variaveis Globais
///
cv_bridge::CvImagePtr image_ptr; // Ponteiro para imagem da camera
bool aquisitando = false, transicao = false, aquisitar_imagem = false, fim_processo = false;
int indice_posicao = 0; // Posicao do vetor de posicoes sendo observada no momento
int contador_nuvem = 0, N = 100; // Quantas nuvens aquisitar em cada parcial
vector<int> posicoes_pan, posicoes_tilt; // Posicoes a serem vasculhadas pelos motores
// Inicia variaveis do motor PAN
double raw_min_pan = 35, raw_max_pan = 4077;
double deg_min_pan =  3, deg_max_pan =  358;
double deg_raw_pan, raw_deg_pan; // Definidas no main
double dentro = 10; // Raio de seguranca que estamos dentro ja [RAW] (pelas contas aproximadas, 1 RAW = 0.08 degrees)
// Inicia variaveis do motor TILT - horizontal da offset para ser o zero
double raw_min_tilt = 2595, raw_hor_tilt = 2150, raw_max_tilt = 1595;
double deg_min_tilt =   39, deg_hor_tilt =    0, deg_max_tilt =  -50;
double deg_raw_tilt, raw_deg_tilt; // Definidas no main
// Classe de processamento de nuvens
ProcessCloud *pc;
// Nuvens de pontos e vetor de nuvens parciais
PointCloud<PointXYZ>::Ptr parcial;
// Vetor com linhas do arquivo NVM
vector<std::string> linhas_nvm;
// Servico para mover os servos
ros::ServiceClient comando_motor;
// Servico para controlar o LED
ros::ServiceClient comando_leds;
// Testando sincronizar subscribers por mutex
Mutex m;

///////////////////////////////////////////////////////////////////////////////////////////
int deg2raw(double deg, std::string motor){
    if(motor == "pan")
        return int((deg - deg_min_pan )*raw_deg_pan  + raw_min_pan);
    else
        return int((deg - deg_min_tilt)*raw_deg_tilt + raw_min_tilt);
}
double raw2deg(int raw, std::string motor){
    if(motor == "pan")
        return (double(raw) - raw_min_pan )*deg_raw_pan  + deg_min_pan;
    else
        return (double(raw) - raw_min_tilt)*deg_raw_tilt + deg_min_tilt - deg_hor_tilt;
}
///////////////////////////////////////////////////////////////////////////////////////////

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
            ROS_WARN("Nuvem %d foi acumulada, processando ...", indice_posicao+1);
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
            pass.setFilterLimits(0, 30); // Z metros de profundidade
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
            // Transformar nuvem de acordo com a posicao dos servos, e pegar pose da camera em consequencia
            float p = raw2deg(posicoes_pan[indice_posicao], "pan"), t = raw2deg(posicoes_tilt[indice_posicao], "tilt");
            Vector3f C;
            Quaternion<float> q;
            pc->transformCloudAndCamServoAngles(cloud_color, p, t, C, q);
            // Salvar dados parciais na pasta Dados_PEPO (ou o nome inserido), no Desktop
            ROS_WARN("Salvando dados de imagem e nuvem da aquisicao %d de %zu ...", indice_posicao+1, posicoes_pan.size());
            string nome_imagem_atual;
            if(indice_posicao+1 < 10){
              nome_imagem_atual = "imagem_00"+std::to_string(indice_posicao+1);
              pc->saveImage(image_ptr->image, nome_imagem_atual);
              pc->saveCloud(cloud_color, "pf_00"+std::to_string(indice_posicao+1));
            } else if(indice_posicao+1 < 100) {
              nome_imagem_atual = "imagem_0"+std::to_string(indice_posicao+1);
              pc->saveImage(image_ptr->image, nome_imagem_atual);
              pc->saveCloud(cloud_color, "pf_0"+std::to_string(indice_posicao+1));
            } else {
              nome_imagem_atual = "imagem_"+std::to_string(indice_posicao+1);
              pc->saveImage(image_ptr->image, nome_imagem_atual);
              pc->saveCloud(cloud_color, "pf_"+std::to_string(indice_posicao+1));
            }
            /// Adicionar ao vetor a linha correspondente do NVM ///
            // Escreve a linha e armazena
            linhas_nvm[indice_posicao] = pc->escreve_linha_imagem((1133.3 + 1121.6)/2, nome_imagem_atual, C, q); // Brio
            //////////////////////
            // Zerar contador de nuvens da parcial
            contador_nuvem = 0;
            // Terminamos o processamento, virar flag
            ROS_WARN("Terminada aquisicao da nuvem %d", indice_posicao+1);
            // Enviar os servos para a proxima posicao, se nao for a ultima ja
            dynamixel_workbench_msgs::JointCommand cmd;
            cmd.request.unit = "raw";
            if(indice_posicao + 1 < posicoes_pan.size()){
                indice_posicao++; // Proximo ponto de observacao
                cmd.request.pan_pos  = posicoes_pan[indice_posicao];
                cmd.request.tilt_pos = posicoes_tilt[indice_posicao];
                if(comando_motor.call(cmd))
                    ROS_INFO("Indo para a posicao %d de %zu totais aquisitar nova nuvem", indice_posicao+1, posicoes_pan.size());
                // Seguranca para evitar balancar muito e capturar com ruido
                sleep(1); // Seguranca para nao capturar nada balancando
            } else { // Se for a ultima, finalizar
                // Voltando para o inicio
                cmd.request.pan_pos  = posicoes_pan[0];
                cmd.request.tilt_pos = posicoes_tilt[0]; // Vai deitar mesmo
                comando_motor.call(cmd);
                ROS_INFO("Aquisitamos todas as nuvens, salvando tudo e indo para a posicao inicial ...");
                // Salvando o NVM final
                ROS_INFO("Salvando arquivo NVM ...");
                pc->compileFinalNVM(linhas_nvm);
                // Finalizando o no e o ROS
                fim_processo = true;
                ROS_WARN("Finalizando tudo, conferir dados salvos.");
            }
            m.unlock();
        } else {
            contador_nuvem++;
        }
    }
}

/// Subscriber dos servos
///
void dynCallback(const nav_msgs::OdometryConstPtr& msg){
    // As mensagens trazem angulos em unidade RAW
    int pan = int(msg->pose.pose.position.x), tilt = int(msg->pose.pose.position.y);
    // Se estiver perto do valor de posicao atual de gravacao, liberar a aquisicao
    if(abs(pan - posicoes_pan[indice_posicao]) <= dentro && abs(tilt - posicoes_tilt[indice_posicao]) <= dentro){
        aquisitando = true;
        posicoes_pan[indice_posicao] = pan; posicoes_tilt[indice_posicao] = tilt; // Para ficar mais preciso ainda na transformacao
        ROS_INFO("Estamos captando na posicao %d ...", indice_posicao+1);
    } else {
        aquisitando = false;
    }
    // Se ja tiver no fim do processo, confere se esta chegando no inicio pra dai desligar os motores
    if(fim_processo){
        ROS_WARN("Estamos voltando ao inicio, %d para PAN e %d para TILT ...", int(abs(pan - raw_min_pan)), int(abs(tilt - raw_min_tilt)));
        // LED fixo - ACABOU
        led_control::LED cmd_led;
        cmd_led.request.led = 1;
        comando_leds.call(cmd_led);
        if(abs(pan - raw_min_pan) <= dentro && abs(tilt - posicoes_tilt[indice_posicao]) <= dentro){
            ROS_WARN("Chegamos ao final, desligando ...");
            system("gnome-terminal -x sh -c 'rosnode kill -a'");
            ros::shutdown();
        }
    }
}

/// Main
///
int main(int argc, char **argv)
{
    ros::init(argc, argv, "scanner_aquisicao");
    ros::NodeHandle nh;
    ros::NodeHandle n_("~");
    ROS_INFO("Iniciando o processo do SCANNER - aguardando servos ...");

    // Pegando o nome da pasta por parametro
    string nome_param;
    n_.param("pasta", nome_param, string("Dados_PEPO"));

    // Apagando pasta atual e recriando a mesma na area de trabalho
    char* home;
    home = getenv("HOME");
    std::string pasta = std::string(home)+"/Desktop/"+nome_param.c_str()+"/";
    system(("rm -r "+pasta).c_str());
    mkdir(pasta.c_str(), 0777);

    sleep(3); // Esperar os motores ligarem e ficarem prontos
    ros::Rate rd(2);

    // Definindo as taxas raw - deg dos servos
    deg_raw_pan  = (deg_max_pan  - deg_min_pan ) / (raw_max_pan  - raw_min_pan ); raw_deg_pan  = 1/deg_raw_pan;
    deg_raw_tilt = (deg_max_tilt - deg_min_tilt) / (raw_max_tilt - raw_min_tilt); raw_deg_tilt = 1/deg_raw_tilt;

    // Preenchendo os vetores de posicao a ser escaneadas
    int step = 20; // [degrees]
    int vistas_tilt = abs(int(deg_max_tilt - deg_min_tilt))/step + 1;
    vector<double> tilts(vistas_tilt);
    for(int j=0; j < vistas_tilt; j++)
        tilts[j] = deg2raw(deg_min_tilt - double(j*step), "tilt");
    int vistas_pan = int(deg_max_pan - deg_min_pan)/step + 1; // Vistas na horizontal
    posicoes_pan.resize(vistas_pan * vistas_tilt);
    posicoes_tilt.resize(posicoes_pan.size());
    omp_set_dynamic(0);
    #pragma omp parallel for num_threads(tilts.size())
    for(int j=0; j < vistas_tilt; j++){
        for(int i=0; i < vistas_pan; i++){
            if(remainder(j, 2) == 0) // Define assim um vai e volta no scanner, ao inves de voltar ao inicio
                posicoes_pan[i + j*vistas_pan] = deg2raw(deg_min_pan + double(i*step), "pan");
            else
                posicoes_pan[i + j*vistas_pan] = deg2raw(deg_max_pan - double(i*step), "pan");
            // Posicoes tilt repetidas vezes para cada volta completa de pan
            posicoes_tilt[i + j*vistas_pan] = int(tilts[j]);
        }
    }

    // Inicia nuvem parcial acumulada a cada passagem do laser
    parcial  = (PointCloud<PointXYZ>::Ptr) new PointCloud<PointXYZ>();
    parcial->header.frame_id  = "pepo";

    // Inicia o vetor de linhas do arquivo NVM
    linhas_nvm.resize(posicoes_pan.size());

    // Inicia servico para mexer os servos
    comando_motor = nh.serviceClient<dynamixel_workbench_msgs::JointCommand>("/joint_command");

    // Inicia servico para acionar o LED
    comando_leds = nh.serviceClient<led_control::LED>("/controle_led");

    // Ligar o LED piscando rapido - MOTORES SE AJUSTANDO
    led_control::LED cmd_led;
    cmd_led.request.led = 2;
    comando_leds.call(cmd_led);

    // Enviando scanner para o inicio
    dynamixel_workbench_msgs::JointCommand cmd;
    cmd.request.unit = "raw";
    cmd.request.pan_pos  = posicoes_pan[0];
    cmd.request.tilt_pos = posicoes_tilt[0];

    while(!comando_motor.call(cmd)){
        ROS_ERROR("Esperando a comunicacao com os servos !! ...");
        ros::spinOnce();
        rd.sleep();
    }
    ROS_INFO("Servos comunicando e indo para a posicao inicial ...");
    sleep(2); // Esperar os servos pararem de balancar

    // Inicia classe de processo de nuvens
    pc = new ProcessCloud(pasta);

    // Subscribers dessincronizados para mensagens de laser, imagem e motores
    ros::Subscriber sub_laser = nh.subscribe("/livox/lidar"                    , 10, laserCallback);
    ros::Subscriber sub_cam   = nh.subscribe("/camera/image_raw"               , 10, camCallback  );
    ros::Subscriber sub_dyn   = nh.subscribe("/dynamixel_angulos_sincronizados",  1, dynCallback  );

    ROS_INFO("Comecando a aquisicao ...");

    // LED piscando lentamente - AQUISITANDO
    cmd_led.request.led = 3;
    comando_leds.call(cmd_led);

    sensor_msgs::PointCloud2 cloud_msg;
    cloud_msg.header.frame_id = "pepo";
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

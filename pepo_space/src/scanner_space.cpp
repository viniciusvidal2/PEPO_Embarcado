﻿/// Includes
///
#include <iostream>
#include <ros/ros.h>
#include <string>
#include <math.h>

#include <std_msgs/Float32MultiArray.h>
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

#include "communication/state.h"

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
// Pasta onde vao ser salvos os arquivos
string pasta;
// Vetores para posicao angular dos servos
vector<int>   pans_raw, tilts_raw; // [RAW]
vector<float> pans_deg, tilts_deg; // [DEG]
// Limites em raw e deg para os servos de pan e tilt
float raw_min_pan = 35, raw_max_pan = 4077;
float deg_min_pan =  3, deg_max_pan =  358;
float raw_min_tilt = 2595, raw_hor_tilt = 2280, raw_max_tilt = 1595  ;
float deg_min_tilt =   28, deg_hor_tilt =    0, deg_max_tilt =  -60.9;
float raw_deg = 11.37777, deg_raw = 1/raw_deg;
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
bool aquisitar_imagem_imu = false, mudando_vista = true, iniciar_laser = false;
// Publicador de imagem, nuvem parcial e odometria
ros::Publisher cl_pub;
ros::Publisher od_pub;
// Classe de processamento de nuvens
ProcessCloud *pc;
// Classe de processamento de imagens
ProcessImages *pi;
// Nuvens de pontos e vetor de nuvens parciais
PointCloud<PointXYZ>::Ptr parcial;
// Valor do servo naquele instante em variavel global para ser acessado em varios callbacks
int pan, tilt;
// Valor dos angulos que vem da IMU, em RAD
float roll, pitch;
// Vetor de odometrias para cada imagem - somente tilt ja devia ser suficiente
vector<int> tilts_imagens_pan_atual;
// Vetor com todos os quaternions para fazer a panoramica
vector<Quaternion<float>> quaternions_panoramica;
// Quantos tilts vamos capturar
int ntilts;
// Ponteiro de cv_bridge para a imagem
cv_bridge::CvImagePtr image_ptr;

// Tempos para artigo
vector<float > tempos_entre_aquisicoes, tempos_colorir, tempos_salvar, tempos_filtrar;
vector<size_t> pontos_nuvem_inicial, pontos_filtro_colorir;

///////////////////////////////////////////////////////////////////////////////////////////
int deg2raw(double deg, string motor){
    if(motor == "pan")
        return int(deg*raw_deg);//int((deg - deg_min_pan )*raw_deg + raw_min_pan);
    else
        return int((deg - deg_min_tilt)*raw_deg + raw_min_tilt);
}
float raw2deg(int raw, string motor){
    if(motor == "pan")
        return float(raw)*deg_raw;//(float(raw) - raw_min_pan )*deg_raw + deg_min_pan;
    else
        return (float(raw) - raw_max_tilt)*deg_raw + deg_max_tilt;
}
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
void saveTimeFiles(){
    // Abre os arquivos todos
    ofstream t_ea(pasta+"tempos_entre_aquisicoes.txt");
    ofstream t_cor(pasta+"tempos_colorir.txt");
    ofstream t_salvar(pasta+"tempos_salvarimagem.txt");
    ofstream t_f(pasta+"tempos_filtrarorigens.txt");

    // Escreve uma linha para cada valor
    if(t_ea.is_open()){
        for(auto t:tempos_entre_aquisicoes){
            t_ea << t;
            t_ea << "\n";
        }
    }
    if(t_cor.is_open()){
        for(auto t:tempos_colorir){
            t_cor << t;
            t_cor << "\n";
        }
    }
    if(t_salvar.is_open()){
        for(auto t:tempos_salvar){
            t_salvar << t;
            t_salvar << "\n";
        }
    }
    if(t_f.is_open()){
        for(auto t:tempos_filtrar){
            t_f << t;
            t_f << "\n";
        }
    }

    // Fecha arquivos
    t_ea.close(); t_cor.close(); t_salvar.close(); t_f.close();
}
///////////////////////////////////////////////////////////////////////////////////////////
void savePointFiles(){
    // Abre os arquivos todos
    ofstream p_ni(pasta+"pontos_nuvem_inicial.txt");
    ofstream p_fc(pasta+"pontos_filtra_colore.txt");

    // Escreve uma linha para cada valor
    if(p_ni.is_open()){
        for(auto p:pontos_nuvem_inicial){
            p_ni << p;
            p_ni << "\n";
        }
    }
    if(p_fc.is_open()){
        for(auto p:pontos_filtro_colorir){
            p_fc << p;
            p_fc << "\n";
        }
    }

    // Fecha arquivos
    p_ni.close(); p_fc.close();
}
///////////////////////////////////////////////////////////////////////////////////////////

/// Callback da camera
///
void camCallback(const sensor_msgs::ImageConstPtr& msg){
    // Aqui ja temos a imagem em ponteiro de opencv, depois de pegar uma desabilitar
    if(aquisitar_imagem_imu)
        image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
}

/// Callback servos
///
void servosCallback(const nav_msgs::OdometryConstPtr &msg_servos){
    // As mensagens trazem angulos em unidade RAW
    pan = int(msg_servos->pose.pose.position.x), tilt = int(msg_servos->pose.pose.position.y);
}

/// Callback imu
///
void imuCallback(const std_msgs::Float32MultiArrayConstPtr &msg_imu){
    // As mensagens trazem angulos em RAD
    if(aquisitar_imagem_imu){
        roll = msg_imu->data.at(0); pitch = msg_imu->data.at(1);
    }
}

/// Callback do laser
///
void laserCallback(const sensor_msgs::PointCloud2ConstPtr &msg_cloud){
    // Se nao estamos processando a nuvem e publicando, nem mudando de vista em pan, captar
    if(!mudando_vista && iniciar_laser){
        // Converter mensagem
        PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
        fromROSMsg(*msg_cloud, *cloud);
        // Acumular nuvem parcial
        *parcial += *cloud;
    }
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

    ///// ENCONTRANDO PU PARA A MAQUINA JETSON
    ///
    int a=0;
    ros::Time tem;
    vector<uint64_t> temps(500);
    for(int k=1; k<temps.size(); k++){
      tem = ros::Time::now();
      for(int i=0; i<10; i++)
        a = 10+10;
      temps[k] = (ros::Time::now() - tem).toNSec();
    }
    float avg = float(accumulate(temps.begin(), temps.end(), 0))/float(temps.size());
    ROS_WARN("TEMPO PU DO EDGE: %.0f", avg);
    cout << endl << endl << endl;

    // Pegando os parametros
    string nome_param;
    int step = 30; // [DEG]
    float inicio_scanner_deg_pan, final_scanner_deg_pan;
    int qualidade; // Qualidade a partir de quanto tempo vamos parar em uma vista aquisitando laser e imagem
    n_.param<string>("pasta", nome_param, string("Dados_PEPO"));
    n_.param<int   >("step" , step      , 30                  );
    n_.param<float >("inicio_pan", inicio_scanner_deg_pan, step/2      );
    n_.param<float >("fim_pan"   , final_scanner_deg_pan , 360 - step/2);
    n_.param<int   >("qualidade" , qualidade             , 2           );
    inicio_scanner_deg_pan = (inicio_scanner_deg_pan ==   0) ? deg_min_pan : inicio_scanner_deg_pan;
    final_scanner_deg_pan  = (final_scanner_deg_pan  == 360) ? deg_max_pan : final_scanner_deg_pan;
    if((final_scanner_deg_pan - inicio_scanner_deg_pan) < step || (final_scanner_deg_pan - inicio_scanner_deg_pan) < 0) final_scanner_deg_pan = inicio_scanner_deg_pan + step;
    switch(qualidade){
        case 1:
            qualidade = 15;
            break;
        case 2:
            qualidade = 25;
            break;
        case 3:
            qualidade = 100;
            break;
    }

    // Servico para avisar se comecou ou acabou
    ros::ServiceClient switch_state_srv = nh.serviceClient<communication::state>("switch_state");
    communication::state srv_msg;
    srv_msg.request.state = 1;
    if(switch_state_srv.call(srv_msg))
        ROS_WARN("Iniciamos o sistema corretamente !");
    else
        ROS_ERROR("Gerenciamento global nao sabe que iniciamos o sistema !");

    // Toda a organizacao em pastas na area de trabalho para o space atual
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

    /// Preenchendo vetor de pan e tilt primeiro para a camera
    ///
    ros::Time tini = ros::Time::now();
    // Pontos de observacao em tilt
    vector<float> tilts_camera_deg {deg_min_tilt, deg_hor_tilt, -30.0f, deg_max_tilt};
    ntilts = tilts_camera_deg.size();
    // Pontos de observacao em pan
    int vistas_pan = int(final_scanner_deg_pan - inicio_scanner_deg_pan)/step + 2; // Vistas na horizontal, somar inicio e final do range
    vector<float> pans_camera_deg;
    for(int j=0; j < vistas_pan-1; j++)
        pans_camera_deg.push_back(inicio_scanner_deg_pan + float(j*step));
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
    ROS_WARN("TEMPO PARA INICIAR TUDO: %zu", (ros::Time::now() - tini).toNSec());
    // Inicia servico para mexer os servos
    comando_motor = nh.serviceClient<dynamixel_workbench_msgs::JointCommand>("/joint_command");

    // Ligar o LED piscando rapido - MOTORES SE AJUSTANDO
    led_control::LED cmd_led;
    cmd_led.request.led = 2;
    comando_leds.call(cmd_led);

    // Iniciando ponteiro de imagem em cv_bridge
    image_ptr = (cv_bridge::CvImagePtr) new cv_bridge::CvImage;

    // Subscribers
    ros::Subscriber sub_cam = nh.subscribe("/camera/image_raw"               , 10, camCallback   );
    ros::Subscriber sub_las = nh.subscribe("/livox/lidar"                    , 10, laserCallback );
    ros::Subscriber sub_dyn = nh.subscribe("/dynamixel_angulos_sincronizados", 10, servosCallback);
    ros::Subscriber sub_imu = nh.subscribe("/imu"                            , 10, imuCallback   );

    // Enviando scanner para o inicio
    cmd.request.unit = "raw";
    cmd.request.pan_pos  = pans_raw[0];
    cmd.request.tilt_pos = tilts_raw[0];

    ros::Rate r(20);
    ROS_WARN("Esperando a IMU !! ...");
    while(sub_imu.getNumPublishers() == 0){
        ros::spinOnce();
        r.sleep();
    }
    ROS_WARN("Esperando a comunicacao com os servos !! ...");
    while(!comando_motor.call(cmd)){
        ros::spinOnce();
        r.sleep();
    }
    ROS_INFO("Servos comunicando e indo para a posicao inicial ...");
    sleep(6); // Esperar os servos pararem de balancar e driver de imagem ligar

    // Inicia classe de processo de nuvens e imagens
    pc = new ProcessCloud (pasta);
    pi = new ProcessImages(pasta);

    // Publicadores
    cl_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_space", 10);
    od_pub = nh.advertise<nav_msgs::Odometry      >("/angle_space", 10);
    ros::Publisher msg_pub = nh.advertise<std_msgs::Float32>("/feedback_scan", 10);
    std_msgs::Float32 msg_feedback;
    msg_feedback.data = 0;
    msg_pub.publish(msg_feedback);

    // Iniciando a nuvem parcial acumulada de cada pan
    parcial = (PointCloud<PointXYZ>::Ptr) new PointCloud<PointXYZ>();
    parcial->header.frame_id  = "map";

    ROS_INFO("Comecando a aquisicao ...");

    // LED piscando lentamente - AQUISITANDO
    cmd_led.request.led = 3;
    comando_leds.call(cmd_led);

    ros::Time tempo_nuvem = ros::Time::now();

    while(ros::ok()){

        // Controlando aqui o caminho dos servos, ate chegar ao final
        if(abs(pan - pans_raw[indice_posicao]) <= dentro && abs(tilt - tilts_raw[indice_posicao]) <= dentro && indice_posicao != pans_raw.size()){

            // Se estavamos mudando de vista, nao estamos mais
            if(mudando_vista) mudando_vista = false;
            sleep(4); // Garantir o servo no lugar

            // Se chegamos na primeira captura, indice 0, podemos comecar a capturar o laser
            if(!iniciar_laser) iniciar_laser = true;
            ROS_INFO("Estamos captando a imagem %d ...", indice_posicao+1);
            // Libera captura da imagem e imu
            aquisitar_imagem_imu = true;
            for(int i=0; i<qualidade; i++){
                r.sleep();
                ros::spinOnce();
            }
            // Chavear a flag
            aquisitar_imagem_imu = false;
            // Pegar o roll para essa vista em pan
            float current_roll;
            if(indice_posicao % ntilts == 0){
//                sleep(2);
                current_roll = 0;
            }
            Mat imagem_temp;
            image_ptr->image.copyTo(imagem_temp);
            // Salvar quaternion para criacao da imagem 360 final
            Matrix3f R360 = pc->euler2matrix(current_roll, -DEG2RAD(raw2deg(tilt, "tilt")), -DEG2RAD(raw2deg(pan, "pan")));
            Quaternion<float> q360(R360);
            quaternions_panoramica.emplace_back(q360);
            // Salvar a imagem na pasta certa
            ros::Time tempo_s = ros::Time::now();
            string nome_imagem_atual;
            if(indice_posicao + 1 < 10)
                nome_imagem_atual = "imagem_00"+std::to_string(indice_posicao+1);
            else if(indice_posicao+1 < 100)
                nome_imagem_atual = "imagem_0" +std::to_string(indice_posicao+1);
            else
                nome_imagem_atual = "imagem_"  +std::to_string(indice_posicao+1);
            pi->saveImage(imagem_temp, nome_imagem_atual);
            tempos_salvar.push_back((ros::Time::now() - tempo_s).toSec());

            // Transformando para o frame da camera
            pc->transformToCameraFrame(parcial);
            ROS_INFO("Filtrando nuvem parcial ...");
            // Excluindo pontos de leituras vazias
            ros::Time tempo_f = ros::Time::now();
            pontos_nuvem_inicial.push_back(parcial->size());
            pc->cleanMisreadPoints(parcial);
            tempos_filtrar.push_back((ros::Time::now() - tempo_f).toSec());
            // Colorir nuvem com todas as imagens
            ROS_INFO("Colorindo nuvem parcial ...");
            ros::Time tempo_c = ros::Time::now();
            PointCloud<PointT>::Ptr parcial_color (new PointCloud<PointT>);
            parcial_color->resize(parcial->size());
#pragma omp parallel for
            for(size_t i=0; i<parcial_color->size(); i++){
                parcial_color->points[i].x = parcial->points[i].x; parcial_color->points[i].y = parcial->points[i].y; parcial_color->points[i].z = parcial->points[i].z;
                parcial_color->points[i].r = 200                 ; parcial_color->points[i].g = 200                 ; parcial_color->points[i].b = 200                 ;
            }
            parcial->clear();

            pc->colorCloudWithCalibratedImage(parcial_color, image_ptr->image, 1);
            tempos_colorir.push_back((ros::Time::now() - tempo_c).toSec());
            pontos_filtro_colorir.push_back(parcial_color->size());
            // Transformando segundo o pitch e roll vindos da imu e pan vindo do servo
            //Vector3f off_laser{0, 0, 0.04};
            transformPointCloud<PointT>(*parcial_color, *parcial_color, Vector3f::Zero(), q360);

            // Publicar tudo para a fog - nuvem e odometria
            ROS_INFO("Publicando nuvem e odometria ...");
            sensor_msgs::PointCloud2 msg_out;
            toROSMsg(*parcial_color, msg_out);
            msg_out.header.stamp = ros::Time::now();
            msg_out.header.frame_id = "map";
            nav_msgs::Odometry odom_cloud_out;
            odom_cloud_out.pose.pose.position.x = -current_roll;
            odom_cloud_out.pose.pose.position.y = DEG2RAD(raw2deg(tilt, "tilt"));
            odom_cloud_out.pose.pose.position.z = pan; // So o pan vai em RAW para a fog melhorar
            odom_cloud_out.pose.pose.orientation.w = pans_raw.size(); // Quantidade total de aquisicoes para o fog ter nocao
            odom_cloud_out.pose.pose.orientation.x = float(ntilts);   // Quantos tilts para a fog se organizar
            odom_cloud_out.pose.pose.orientation.y = float(indice_posicao);  // Posicao atual na aquisicao
            odom_cloud_out.header.stamp    = msg_out.header.stamp;
            odom_cloud_out.header.frame_id = msg_out.header.frame_id;
            od_pub.publish(odom_cloud_out);
            cl_pub.publish(msg_out);
            // Zerando parcial para proxima vista em pan e vetor de imagens
            parcial_color->clear();
            // Vamos mudar de waypoints, segurar a aquisicao
            mudando_vista = true;

            tempos_entre_aquisicoes.push_back((ros::Time::now() - tempo_nuvem).toSec());
            tempo_nuvem = ros::Time::now();
            // Enviar para a proxima posicao
            if(indice_posicao + 1 < pans_raw.size()){

                indice_posicao++; // Proximo ponto de observacao
                cmd.request.pan_pos  = pans_raw[indice_posicao];
                cmd.request.tilt_pos = tilts_raw[indice_posicao];
                if(comando_motor.call(cmd))
                    ROS_INFO("Indo para a posicao %d de %zu totais aquisitar nova imagem ...", indice_posicao+1, pans_raw.size());
                // Fala a porcentagem do total que ja resolvemos
                msg_feedback.data = 100.0*float(indice_posicao)/float(pans_raw.size());
                msg_pub.publish(msg_feedback);

            } else { // Se for a ultima, finalizar

                indice_posicao++;
                ROS_INFO("Aquisitamos tudo, enviando para posicao inicial novamente ...");
                cmd_led.request.led = 1; // LED continuo
                comando_leds.call(cmd_led);
                cmd.request.pan_pos  = pans_raw[0]; // Quase no inicio, pra quando ligar dar uma mexida
                cmd.request.tilt_pos = raw_hor_tilt;
                comando_motor.call(cmd);
                // Ja chegamos no 100 pct
                msg_feedback.data = 100.0;
                msg_pub.publish(msg_feedback);

                // Criar a 360 crua
                ROS_INFO("Processando imagem 360 ...");
                pi->estimateRaw360(quaternions_panoramica, pc->getFocuses(1));
                saveTimeFiles();
                savePointFiles();
                ROS_INFO("Processado e finalizado o Scan.");

                // Avisar ao gerenciador global que acabamos
                sleep(8);
                srv_msg.request.state = 0;
                if(switch_state_srv.call(srv_msg))
                    ROS_INFO("Terminamos a aquisicao corretamente.");
                // Mata todos os nos que estao rodando
                system("rosnode kill camera imu_node livox_lidar_publisher multi_port_pepo scanner_space");

            }
        } // Fim if estamos dentro do waypoint

        // Roda o loop de ROS
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

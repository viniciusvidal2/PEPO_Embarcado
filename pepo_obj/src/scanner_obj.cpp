/// Includes
///
#include <iostream>
#include <ros/ros.h>
#include <ros/service_server.h>
#include <string>
#include <math.h>

#include <std_msgs/Float32.h>
#include <livox_ros_driver/CustomMsg.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include "../../libraries/include/processcloud.h"
#include "../../libraries/include/processimages.h"
#include "pepo_obj/comandoObj.h"

#include "led_control/LED.h"
#include "communication/state.h"

#include <dynamixel_workbench_msgs/DynamixelCommand.h>
#include <dynamixel_workbench_msgs/DynamixelInfo.h>
#include <dynamixel_workbench_msgs/JointCommand.h>
#include <dynamixel_workbench_msgs/DynamixelState.h>

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
ros::Time time_im_ref;
// Flags de controle
bool aquisitando = false, aquisitar_imagem = false, fim_processo = false;
int contador_nuvem = 0, N = 40; // Quantas nuvens aquisitar em cada parcial
// Classe de processamento de nuvens
ProcessCloud *pc;
ProcessImages *pi;
// Publicador da nuvem de pontos que vai criar a imagem virtual para o usuario
ros::Publisher cloud_pub;
// Nuvem de pontos parciais
PointCloud<PointXYZ>::Ptr parcial;
PointCloud<PointT>::Ptr acc;
// Testando sincronizar subscribers por mutex
Mutex m;
// Contador de aquisicoes - usa tambem para salvar no lugar certo
int cont_aquisicao = 0;
int cont_imagem_virtual = 0;
// Publisher para feedback
ros::Publisher feedback_pub;
// Odometria vinda da LOAM
//Quaternionf qloam, qref;
//Vector3f tloam, tref;
Matrix4f Tref  = Matrix4f::Identity();
Matrix4f Tloam = Matrix4f::Identity();
Matrix4f Tcamframe = Matrix4f::Identity();
ros::Time stamp_ref_loam, stamp_ref_laser;
ros::Subscriber sub_loam;
// Controle se servos estao travados - se existe o no publicando
bool servos_locked = false;
int stab_servo = 0;
// Servico para mover os servos
ros::ServiceClient comando_motor;
dynamixel_workbench_msgs::JointCommand cmd;
// Cameras para o sfm
vector<string> cameras;

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
void do_sleep(float sec){
    ros::Rate r(10);
    for(int i=0; i<int(10*sec); i++){
        r.sleep();
        ros::spinOnce();
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
        // Checar contra maior variancia ou o tempo para tentar pegar a mais recente possivel alem de a menos borrada
        if( s[0] > max_var || abs((msg->header.stamp - time_im_ref).toSec()) > 1 ){
            image_ptr->image.copyTo(min_blur_im);
            max_var = s[0];
            time_im_ref = msg->header.stamp;
        }
    }
}

/// Callback odometria LOAM
///
void loamCallback(const nav_msgs::OdometryConstPtr& msg){
    // Atualizar sempre a odometria, guardar o ultimo stamp para sincronizar com o momento de salvar
    // no callback do laser
    Quaternionf qloam;
    Vector3f tloam;
    qloam.w() = msg->pose.pose.orientation.w;
    qloam.x() = msg->pose.pose.orientation.x;
    qloam.y() = msg->pose.pose.orientation.y;
    qloam.z() = msg->pose.pose.orientation.z;
    tloam << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    Tloam.block<3,3>(0, 0) = qloam.matrix();
    Tloam.block<3,1>(0, 3) = tloam;
    Tloam = Tcamframe*Tloam*Tcamframe.inverse();
    stamp_ref_loam = msg->header.stamp;
}

/// Callback dos servos
///
void servosCallback(const nav_msgs::OdometryConstPtr& msg){

    cmd.request.unit = "raw";
    cmd.request.pan_pos  = msg->pose.pose.position.x;
    cmd.request.tilt_pos = msg->pose.pose.position.y;

    // Se os servos estao travados, libera leitura de imagem e laser nos callbacks
    if(!servos_locked)
        stab_servo++;
    if(!servos_locked && stab_servo == 6){
        int ret = comando_motor.call(cmd);
        do_sleep(3);
        servos_locked = true;
        stab_servo = 0;
        // Libera tambem a camera aqui e o tempo que comecamos a aquisicao
        aquisitar_imagem = true;
        time_im_ref = msg->header.stamp;
    }

}

/// Callback do laser
///
void laserCallback(const livox_ros_driver::CustomMsgConstPtr& msg){
    // Ler a mensagem customizada do livox e converter para point cloud
    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>());
    cloud->resize(msg->point_num);
#pragma omp parallel for
    for(size_t i = 0; i < msg->point_num; ++i){
      PointXYZ pt;
      pt.x = msg->points[i].x; pt.y = msg->points[i].y; pt.z = msg->points[i].z;
      cloud->points[i] = pt;
    }
    pc->cleanMisreadPoints(cloud);
    // Encaminha para o no de comunicacao que cria imagem virtual
    sensor_msgs::PointCloud2 comm_msg;
    toROSMsg(*cloud, comm_msg);
    cloud_pub.publish(comm_msg);

    // Marca o stamp em que comecamos a aquisicao - adianta a correspondencia com odometria
    if(aquisitando && contador_nuvem == 0)
        stamp_ref_laser = msg->header.stamp;

    //ROS_WARN("Temos aqui a diferenca de stamps  LASER: %.2f   ODOM: %.2f   DIFF: %.2f", msg->header.stamp.toSec(), stamp_ref_loam.toSec(), (msg->header.stamp - stamp_ref_loam).toSec());

    // Acumular na nuvem total por N vezes quando aquisitando
    if(aquisitando && servos_locked){
        *parcial += *cloud;
        // Atualizar contador de nuvem
        contador_nuvem++;
        // Matriz de Odometria atual (se for o momento)
        Matrix4f Todo;
        // Se total acumulado, travar o resto e trabalhar
        if(contador_nuvem == N){
            cont_aquisicao++;
            ROS_WARN("Aquisicao %d foi acumulada, processando ...", cont_aquisicao);
            // Vira a variavel de controle de recebimento de imagens e da nuvem
            aquisitar_imagem = false;
            aquisitando = false;
            // Injetando cor na nuvem
            PointCloud<PointT>::Ptr cloud_color (new PointCloud<PointT>());
            cloud_color->resize(parcial->size());
#pragma omp parallel for
            for(size_t i=0; i < cloud_color->size(); i++){
                cloud_color->points[i].r = 200; cloud_color->points[i].g = 200; cloud_color->points[i].b = 200;
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
            pass.setFilterLimits(0, 40); // Z metros de profundidade
            pass.filter(*cloud_color);
            // Colorir pontos com calibracao default para visualizacao rapida
            ROS_WARN("Colorindo nuvem para salvar com parametros default ...");
            pc->colorCloudWithCalibratedImage(cloud_color, image_ptr->image, 1);
            pc->cleanNotColoredPoints(cloud_color);

            // Aguardar atualizacao da odometria
            float remaining_time = abs((stamp_ref_laser - stamp_ref_loam).toSec()), stamp_dif = remaining_time;
            std_msgs::Float32 msg_feedback;
            //ROS_INFO("Temos aqui a diferenca de stamps  LASER: %.2f   ODOM: %.2f   DIFF: %.2f", stamp_ref_laser.toSec(), stamp_ref_loam.toSec(), (stamp_ref_laser - stamp_ref_loam).toSec());
            while(abs((stamp_ref_laser - stamp_ref_loam).toSec()) >= 1 && stamp_ref_laser.toSec() > stamp_ref_loam.toSec()){
//                ROS_WARN("AGUARDANDO aqui a diferenca de stamps  LASER: %.2f   ODOM: %.2f   DIFF: %.2f", stamp_ref_laser.toSec(), stamp_ref_loam.toSec(), (stamp_ref_laser - stamp_ref_loam).toSec());
//                if(abs(stamp_dif - (stamp_ref_laser - stamp_ref_loam).toSec()) > 1){
//                    stamp_dif = abs((stamp_ref_laser - stamp_ref_loam).toSec());
//                    msg_feedback.data = ((75 + 25*(1 - stamp_dif/remaining_time)) < 100) ? 75 + 25*(1 - stamp_dif/remaining_time) : 99;
//                    feedback_pub.publish(msg_feedback);
//                }
                do_sleep((stamp_ref_laser - stamp_ref_loam).toSec());
                ros::spinOnce();
            }

            // Renovar a odometria com a referencia anterior e transformar a nuvem
            Todo = Tref*Tloam;
            transformPointCloud<PointT>(*cloud_color, *cloud_color, Todo);

            // Salvar dados parciais na pasta no Desktop
            ROS_WARN("Salvando dados de imagem e nuvem da aquisicao %d ...", cont_aquisicao);
            Mat im2save;
            min_blur_im.copyTo(im2save);
            min_blur_im.release();
            string nome_imagem;
            if(cont_aquisicao < 10){
                nome_imagem = "imagem_00"+std::to_string(cont_aquisicao);
                pc->saveCloud(cloud_color, "pf_00"+std::to_string(cont_aquisicao));
            } else if(cont_aquisicao < 100) {
                nome_imagem = "imagem_0"+std::to_string(cont_aquisicao);
                pc->saveCloud(cloud_color, "pf_0"+std::to_string(cont_aquisicao));
            } else {
                nome_imagem = "imagem_"+std::to_string(cont_aquisicao);
                pc->saveCloud(cloud_color, "pf_"+std::to_string(cont_aquisicao));
            }
            pi->saveImage(im2save, nome_imagem);

            // Adicionar linha para o sfm
            Vector3f tsfm = Todo.block<3,3>(0, 0)*(-pc->gettCam()) + Todo.block<3,1>(0, 3);
            cameras.emplace_back(pc->escreve_linha_sfm(nome_imagem+".png", Todo.inverse().block<3,3>(0, 0), -Todo.inverse().block<3,3>(0, 0)*tsfm));

            max_var = 0; // Liberando a imagem para a proxima captura
            ROS_WARN("Terminada aquisicao da nuvem %d", cont_aquisicao);

            // Acumular nuvem raw para o usuario
            VoxelGrid<PointT> voxel;
            voxel.setLeafSize(0.03, 0.03, 0.03);
            voxel.setInputCloud(cloud_color);
            voxel.filter(*cloud_color);
            *acc += *cloud_color;

        }

        // Falar a porcentagem da aquisicao para o usuario
        std_msgs::Float32 msg_feedback;
        msg_feedback.data = 15 + 60*float(contador_nuvem)/float(N);
        if(contador_nuvem == N){
            // Armazenar odometria de Referencia
            Tref = Todo;
            // Religar odometria
            int ans = system("rosnode kill imu_process laserMapping laserOdometry livox_repub scanRegistration");
            do_sleep(3);
            ans = system("nohup roslaunch loam_horizon loam_livox_horizon_imu.launch rviz:=false &");
            Tloam = Matrix4f::Identity();
            while(Tloam == Matrix4f::Identity()){ // Enquanto nao comecarem a vir dados
                do_sleep(0.2);
                ros::spinOnce();
            }
            // Matar os servos ao terminar e religar odometria
            ans = system("rosnode kill multi_port_cap");
            msg_feedback.data = 100.0;
            feedback_pub.publish(msg_feedback);
            do_sleep(1);
            msg_feedback.data = 1;
            feedback_pub.publish(msg_feedback);
            // Reseta o contador de nuvens aqui, mais seguro
            contador_nuvem = 0;
        } else {
            feedback_pub.publish(msg_feedback);
        }

    }
}

/// Servico para controle de aquisicao
///
bool capturar_obj(pepo_obj::comandoObj::Request &req, pepo_obj::comandoObj::Response &res){
    if(req.comando == 1){ // Havera mais uma nova aquisicao

        // Liga os servos para nao moverem ali daquela posicao enquanto escaneia
        int ans = system("nohup roslaunch dynamixel_workbench_controllers multi_port_cap.launch &");
        servos_locked = false;

        // Mensagem fake para o aplicativo ser notificado de aquisicao
        std_msgs::Float32 msg_feedback;
        msg_feedback.data = 5;
        feedback_pub.publish(msg_feedback);
        // Tempo para a odometria se estabilizar
        do_sleep(2);
        msg_feedback.data = 10;
        feedback_pub.publish(msg_feedback);
        do_sleep(2);
        msg_feedback.data = 15;
        feedback_pub.publish(msg_feedback);

        aquisitando = true;

        ROS_INFO("Realizando aquisicao na posicao %d ...", cont_aquisicao+1);
        res.result = 1;

    } else if (req.comando == 2) { // Acabamos de aquisitar

        // Simplificar ali a nuvem acumulada
        VoxelGrid<PointT> voxel;
        voxel.setLeafSize(0.03, 0.03, 0.03);
        voxel.setInputCloud(acc);
        voxel.filter(*acc);
        pc->saveCloud(acc, "acumulada");
        // Salvar SFM final na pasta
        pc->compileFinalSFM(cameras);

        // Esperar e finalizar o processo
        do_sleep(5);
        res.result = 1;
        fim_processo = true;
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

    // Iniciar a matriz de rotacao para o frame da camera
    Quaternionf qcamframe( AngleAxisf(M_PI/2, Vector3f::UnitZ()) * AngleAxisf(-M_PI/2, Vector3f::UnitY()) );
    Tcamframe.block<3,3>(0, 0) = qcamframe.matrix();

    // Inicia nuvem parcial acumulada a cada passagem do laser
    parcial = (PointCloud<PointXYZ>::Ptr) new PointCloud<PointXYZ>();
    parcial->header.frame_id  = "pepo";
    acc = (PointCloud<PointT>::Ptr) new PointCloud<PointT>();
    acc->header.frame_id  = "pepo";

    // Inicia classe de processo de nuvens
    pc = new ProcessCloud(pasta);
    pi = new ProcessImages(pasta);

    // Subscribers dessincronizados para mensagens de laser, loam e imagem
    ros::Subscriber sub_laser = nh.subscribe("/livox/lidar"       , 10 , laserCallback);
    ros::Subscriber sub_cam   = nh.subscribe("/camera/image_raw"  , 10 , camCallback  );
    ros::Subscriber sub_dyn   = nh.subscribe("/dynamixel_angulos_sincronizados", 2, servosCallback);
    sub_loam = nh.subscribe("/aft_mapped_to_init", 100, loamCallback );

    // Ver se realmente ha odometria
    ros::Rate r(2);
    while(sub_loam.getNumPublishers() < 1 && sub_cam.getNumPublishers() < 1){
        ROS_INFO("Aguardando odometria segura vinda da LOAM ...");
        r.sleep();
    }
    // Aguarda se realmente os dados de odometria estao sendo calculados
    do_sleep(3);
    while(Tloam == Matrix4f::Identity()){
        r.sleep();
        ros::spinOnce();
    }

    // Inicia servidor que recebe o comando sobre como proceder com a aquisicao
    ros::ServiceServer procedimento = nh.advertiseService("/capturar_obj", capturar_obj);

    // Publicadores
    feedback_pub = nh.advertise<std_msgs::Float32>("/feedback_scan", 10);

    // Servico do motor
    comando_motor = nh.serviceClient<dynamixel_workbench_msgs::JointCommand>("/joint_command");

    // Publisher da nuvem
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_virtual_image", 10);

    ROS_INFO("Comecando a aquisicao ...");
    std_msgs::Float32 msg_feedback;
    msg_feedback.data = 1.0; // Para a camera liberar no aplicativo
    for(int i=0; i<5; i++){
        feedback_pub.publish(msg_feedback);
        r.sleep();
    }
    while(ros::ok()){
        r.sleep();
        ros::spinOnce();

        if(fim_processo){
            int ans = system("rosnode kill multi_port_cap");
            ans = system("rosnode kill camera imagem_lr_app livox_lidar_publisher scanner_obj imu_process laserMapping laserOdometry livox_repub scanRegistration");
            ros::shutdown();
            break;
        }
    }

    return 0;
}

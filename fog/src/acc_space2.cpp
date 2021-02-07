/// Includes
///
#include <iostream>
#include <ros/ros.h>
#include <string>
#include <math.h>

#include <std_msgs/Float32.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "../../libraries/include/processcloud.h"
//#include "../../libraries/include/registerobjectoptm.h"

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
// Limites em raw e deg para os servos de pan e tilt
double raw_min_pan = 35, raw_max_pan = 4077;
double deg_min_pan =  3, deg_max_pan =  358;
float raw_min_tilt = 2595.0 , raw_hor_tilt = 2280.0, raw_max_tilt = 1595.0 ;
float deg_min_tilt =   28.0, deg_hor_tilt =    0.0, deg_max_tilt =  -60.20;
float raw_deg_pan, deg_raw_pan, raw_deg_tilt, deg_raw_tilt;
// Publicador de feedback
ros::Publisher msg_pub;
// Classe de processamento de nuvens
ProcessCloud *pc;
//RegisterObjectOptm *roo;
// Nuvens de pontos acumulada e anterior
PointCloud<PointT>::Ptr acc;
//PointCloud<PointTN>::Ptr anterior;
//PointCloud<PointTN>::Ptr parcial;
//PointCloud<PointTN>::Ptr parcial_esq_anterior; // Parte esquerda ja filtrada e armazenada para otimizar o algoritmo
// Nome da pasta que vamos trabalhar
string pasta;
// Poses das cameras para aquela aquisicao [DEG]
vector<float> pan_cameras, pitch_cameras, roll_cameras;
// Vetor com linhas do arquivo NVM
vector<string> linhas_sfm;
// Quantos tilts estao ocorrendo por pan, e contador de quantos ja ocorreram
int ntilts = 4, contador_nuvens = 0;
// Parametros para filtros
float voxel_size, depth;
int filter_poli;
// Braco do centro ao laser
Vector3f off_laser{0, 0, 0.056};

///////////////////////////////////////////////////////////////////////////////////////////
int deg2raw(float deg, string motor){
    if(motor == "pan")
        return int(deg*raw_deg_pan);// int((deg - deg_min_pan )*raw_deg_pan  + raw_min_pan);
    else
        return int((deg - deg_min_tilt)*raw_deg_tilt + raw_min_tilt);
}
float raw2deg(int raw, string motor){
    if(motor == "pan")
        return float(raw)*deg_raw_pan;// (float(raw) - raw_min_pan )*deg_raw_pan  + deg_min_pan;
    else
        return (float(raw) - raw_max_tilt)*deg_raw_tilt + deg_max_tilt;
}
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
    // Salva a parcial daquela vista
    ROS_INFO("Salvando a nuvem do juliano ...");
    if(n < 10){
        pc->saveCloud(cloud, "c_00"+std::to_string(n));
    } else if(n < 100) {
        pc->saveCloud(cloud, "c_0" +std::to_string(n));
    } else {
        pc->saveCloud(cloud, "c_"  +std::to_string(n));
    }
}
///////////////////////////////////////////////////////////////////////////////////////////

/// Callback do laser e odometria sincronizado
///
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg_cloud, const nav_msgs::OdometryConstPtr& msg_angle){
    // As mensagens trazem angulos em unidade RAD, exceto pan
    roll_cameras.push_back(msg_angle->pose.pose.position.x);
    pan_cameras.push_back(DEG2RAD(raw2deg(int(msg_angle->pose.pose.position.z), "pan")));
    pitch_cameras.push_back(msg_angle->pose.pose.position.y);
    // Atualiza a quantidade de tilts que estamos esperando
    ntilts = int(msg_angle->pose.pose.orientation.x);

    // Contador de qual nuvem estamos
    int cont_aquisicao = msg_angle->pose.pose.orientation.y + 1;

    // Converter mensagem em nuvem
    PointCloud<PointT>::Ptr cloud (new PointCloud<PointT>);
    fromROSMsg(*msg_cloud, *cloud);

    // Salva as nuvens temporarias
    saveTempCloud(cloud, cont_aquisicao);

    *acc += *cloud;


    // Fazendo processos finais
    if(cont_aquisicao >= msg_angle->pose.pose.orientation.w){
        ROS_INFO("Processando todo o SFM otimizado ...");
        for(int i=0; i<pan_cameras.size(); i++){
            // Calcula a matriz de rotacao da camera
            float r = -roll_cameras[i], t = -pitch_cameras[i], p = -pan_cameras[i]; // [RAD]
            Matrix3f Rp = pc->euler2matrix(0, 0, -p);
            Matrix3f Rcam = pc->euler2matrix(0, t, p).inverse();

            // Calcula centro da camera
            Vector3f C = Rp*off_laser;
            C = -Rcam.transpose()*pc->gettCam() + C;

            // Calcula vetor de translacao da camera por t = -R'*C
            Vector3f tcam = C;

            // Escreve a linha e anota no vetor de linhas SFM
            string nome_imagem;
            if(i+1 < 10)
                nome_imagem = "imagem_00"+std::to_string(i+1)+".png";
            else if(i+1 < 100)
                nome_imagem = "imagem_0" +std::to_string(i+1)+".png";
            else
                nome_imagem = "imagem_"  +std::to_string(i+1)+".png";
            linhas_sfm.push_back(pc->escreve_linha_sfm(nome_imagem, Rcam, tcam));
        }
        ROS_INFO("Salvando SFM e planta baixa final ...");
        pc->compileFinalSFM(linhas_sfm);
        Mat blueprint;
        float side_area = 20, side_res = 0.04;
        pc->blueprint(acc, side_area, side_res, blueprint);
        acc->clear();

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

    // Pegando o nome da pasta por parametro
    string nome_param;
    n_.param<string>("pasta", nome_param , string("Dados_PEPO"));
    n_.param<float >("vs"   , voxel_size , 2    );
    n_.param<float >("df"   , depth      , 50   );
    n_.param<int   >("fp"   , filter_poli, 1    );

    char* home;
    home = getenv("HOME");
    // Checando ultima pasta criada, pegar ela
    pasta = string(home)+"/Desktop/ambientes/"+nome_param;
    pasta = find_current_folder(pasta + "/scan") + "/";

    // Inicia classe de processo de nuvens
    pc  = new ProcessCloud(pasta);
    //  roo = new RegisterObjectOptm();

    // Calculando taxas exatas entre deg e raw para os motores de pan e tilt
    deg_raw_pan  = 0.08764648;
    deg_raw_tilt = deg_raw_pan;
    raw_deg_pan  = 1.0/deg_raw_pan ;
    raw_deg_tilt = 1.0/deg_raw_tilt;

    // Iniciando a nuvem parcial acumulada de cada pan
    acc = (PointCloud<PointT>::Ptr) new PointCloud<PointT>();
    acc->header.frame_id = "map";
    //  anterior = (PointCloud<PointTN>::Ptr) new PointCloud<PointTN>();
    //  anterior->header.frame_id = "map";
    //  parcial = (PointCloud<PointTN>::Ptr) new PointCloud<PointTN>();
    //  parcial->header.frame_id = "map";
    //  parcial_esq_anterior = (PointCloud<PointTN>::Ptr) new PointCloud<PointTN>();
    //  parcial_esq_anterior->header.frame_id = "map";

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

    ROS_INFO("Comecando a reconstrucao do space ...");

    // Aguardar todo o processamento e ir publicando
    ros::spin();

    return 0;
}

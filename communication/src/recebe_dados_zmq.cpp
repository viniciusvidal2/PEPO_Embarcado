#include <ros/ros.h>
#include <dirent.h>
#include <errno.h>
#include <ostream>
#include <istream>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../msgs/imagem.pb.h"
#include "../msgs/nuvem.pb.h"
#include "../msgs/arquivos.pb.h"
#include "../msgs/nvm.pb.h"
#include "google/protobuf/io/coded_stream.h"
#include "google/protobuf/io/zero_copy_stream.h"

#include <zmq.hpp>
#include <zmq_utils.h>

using namespace ArquivosMsgProto;
using namespace ImagemMsgProto;
using namespace NuvemMsgProto;
using namespace NVMMsgProto;
using namespace pcl;
using namespace pcl::io;
using namespace cv;
using namespace std;
using namespace zmq;

typedef PointXYZRGB PointT;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "recebe_dados_zmq");
  ros::NodeHandle nh;
  ros::NodeHandle n_("~");

  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

  // Diretorio raiz - Desktop por enquanto
  char* home;
  home = getenv("HOME");
  string root = string(home)+"/Desktop/";

  /////////////////////////////////////////////
  /// Ligar Subscriber ZMQ do tipo PULL e
  /// confirmador de recebimento
  ///

  // Iniciando o contexto e subscriber para as mensagens na porta TCP correta
  ROS_WARN("LIGANDO CONTEXTO E SUBSCRIBER ZMQ, PORTA 5558");
  context_t ctx{1};
  socket_t receiver(ctx, ZMQ_PULL); // Tipo PULL para aguardar a mensagem e nao perder nada enviado
  socket_t confirmer(ctx, ZMQ_PUSH); // Vai responder toda vez que chegar uma mensagem, ai sim pode enviar a proxima
  string ip;
  n_.param("ip_pepo", ip, string("192.168.0.101"));
  receiver.connect("tcp://"+ip+":5558");
  confirmer.bind("tcp://*:5557");
  // Prepara a mensagem de confirmacao de dados recebidos
  string confirmer_message_string = "1";
  message_t confirmer_message(confirmer_message_string.length());
  memcpy(confirmer_message.data(), confirmer_message_string.data(), confirmer_message_string.length());

  /////////////////////////////////////////////
  /// Receber o cabecalho com nome da pasta, imagens
  /// e nuvens a receber
  ///

  message_t cabecalho_zmq;
  ROS_INFO("Recebendo cabecalho sobre os dados que virao ...");
  receiver.recv(&cabecalho_zmq);
  ROS_INFO("Cabecalho recebido. Recebendo dados ...");
  string buffer_cabecalho(static_cast<char *>(cabecalho_zmq.data()), cabecalho_zmq.size());
  Arquivos cabecalho_proto;
  cabecalho_proto.ParseFromString(buffer_cabecalho);
  cabecalho_proto.DebugString(); // Mostrar quantas imagens e nuvens virao
  confirmer.send(confirmer_message);

  // Criando pasta a partir do nome vindo na mensagem de cabecalho
  struct stat buffer;
  std::string pasta = root + cabecalho_proto.pasta() + "_recv/";
  system(("rm -r "+pasta).c_str()); // Remove se ja ha algo assim no local
  if(stat(pasta.c_str(), &buffer))  // Checa se nao tem, se positivo pode criar
    mkdir(pasta.c_str(), 0777);
  ROS_INFO("Pasta de recebimento %s criada e pronta para receber arquivos.", pasta.c_str());

  /////////////////////////////////////////////
  /// Recebendo imagens, convertendo o protobuf e
  /// limpando a memoria a cada mensagem recebida
  ///

  // Criando mensagem para cada imagem
  message_t imagem_zmq;
  // Para cada imagem
  for(int i=0; i < cabecalho_proto.imagens(); i++){
    ROS_INFO("Recebendo imagem %d de %d ...", i+1, cabecalho_proto.imagens());
    receiver.recv(&imagem_zmq);
    string temp_img(static_cast<char *>(imagem_zmq.data()), imagem_zmq.size());
    Imagem img_proto;
    img_proto.ParseFromString(temp_img);
    ROS_INFO("Convertendo a imagem %d de %d, com nome %s ...", i+1, cabecalho_proto.imagens(), img_proto.name().c_str());
    // Criando a matriz de opencv
    Mat im(img_proto.height(), img_proto.width(), CV_8UC3, Scalar(0, 0, 0));
    // Escrever cada pixel no lugar certo
    #pragma omp parallel for
    for(int p=0; p<img_proto.pixels_size(); p++){
      // Pega cor do pixel
      Vec3b cor;
      cor[0] = img_proto.pixels(p).b(); cor[1] = img_proto.pixels(p).g(); cor[2] = img_proto.pixels(p).r();
      // Pega posicao do pixel
      int u = img_proto.pixels(p).u(), v = img_proto.pixels(p).v();
      // Anota isso no lugar certo ali na imagem
      im.at<Vec3b>(Point(u, v)) = cor;
    }
    // Salvar a imagem como nome especificado
    imwrite(pasta+img_proto.name(), im);
    // Confirmar que recebemos e ja processamos
    confirmer.send(confirmer_message);
    // Limpar mensagem- redundancia
    temp_img.clear(); im.release();
  }

  /////////////////////////////////////////////
  /// Recebendo nuvens e processando o
  /// protobuf, limpando memoria apos cada recebimento
  ///

  message_t nuvem_zmq;
  // Para cada nuvem
  for(int i=0; i < cabecalho_proto.nuvens(); i++){
    ROS_INFO("Recebendo nuvem %d de %d ...", i+1, cabecalho_proto.nuvens());
    receiver.recv(&nuvem_zmq);
    string temp_cloud(static_cast<char *>(nuvem_zmq.data()), nuvem_zmq.size());
    // Convertendo a nuvem para protobuf
    Nuvem nuvem_proto;
    nuvem_proto.ParseFromString(temp_cloud);
    ROS_INFO("Convertendo a nuvem %d de %d, com nome %s ...", i+1, cabecalho_proto.nuvens(), nuvem_proto.name().c_str());
    // Criando a nuvem em pcl
    PointCloud<PointT>::Ptr cloud (new PointCloud<PointT>);
    cloud->resize(nuvem_proto.size());
    // Escrever cada ponto no lugar certo
    #pragma omp parallel for
    for(int p=0; p<nuvem_proto.size(); p++){
      // Escreve o ponto como veio na mensagem
      PointT ponto;
      ponto.x = nuvem_proto.pontos(p).x(); ponto.y = nuvem_proto.pontos(p).y(); ponto.z = nuvem_proto.pontos(p).z();
      ponto.r = nuvem_proto.pontos(p).r(); ponto.g = nuvem_proto.pontos(p).g(); ponto.b = nuvem_proto.pontos(p).b();
      // Coloca no local na nuvem
      cloud->points[p] = ponto;
    }
    // Salvar a nuvem na pasta com o nome especificado
    savePLYFileASCII<PointT>(pasta+nuvem_proto.name(), *cloud);
    temp_cloud.clear(); cloud->clear();
    // Confirmar que recebemos e ja processamos
    confirmer.send(confirmer_message);
  }

  /////////////////////////////////////////////
  /// Recebendo arquivo NVM
  ///

  message_t nvm_zmq;
  ROS_INFO("Recebendo arquivo NVM ...");
  receiver.recv(&nvm_zmq);
  string buffer_nvm(static_cast<char *>(nvm_zmq.data()), nvm_zmq.size());
  confirmer.send(confirmer_message);
  ROS_INFO("Escrevendo arquivo NVM ...");
  NVM nvm_proto;
  nvm_proto.ParseFromString(buffer_nvm);
  ofstream file(pasta+nvm_proto.name());
  if(file.is_open()){
    // Escrever cabecalho
    file << "NVM_V3\n\n";
    // Para cada linha na mensagem, escrever no arquivo de saida
    for(int i=0; i<nvm_proto.nlinhas(); i++)
      file << nvm_proto.linhas(i)+"\n";
  }
  file.close();

  // Finalizando o contexto ZMQ
  ROS_INFO("Tudo foi recebido e processado. Processo concluido.");
  receiver.close();
  confirmer.close();

  ros::spinOnce();

  return 0;
}

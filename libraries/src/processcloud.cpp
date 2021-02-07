#include "../include/processcloud.h"

/////////////////////////////////////////////////////////////////////////////////////////////////
ProcessCloud::ProcessCloud(string _pasta):pasta(_pasta){
//    // Matriz intrinseca com imagem em Full HD
//    K1 << 1427.1  ,   -0.063, 987.9,
//             0.041, 1449.4  , 579.4,
//             0    ,    0    ,   1  ;
    // Matriz intrinseca com imagem em HD
    K1 << 951.4  ,   -0.063, 658.6,
             0.041, 966.2  , 386.6,
             0    ,    0    ,   1 ;
    // Matriz intrinseca com imagem em resolucao simplificada por 4
    K4 << 375.29  ,   0.0472, 245.18,
            0.0157, 374.50  , 142.36,
            0     ,   0     ,   1   ;
    // Matriz extrinseca com imagem em Full HD
    Rt1.resize(3, 4);
    Rt1 << 1, 0, 0,  0.0077,
           0, 1, 0,  0.0329,
           0, 0, 1,  0.0579;
    // Matriz extrinseca com imagem em resolucao simplificada por 4
    Rt4.resize(3, 4);
    Rt4 << 1, 0, 0,  0.0226,
           0, 1, 0,  0.0938,
           0, 0, 1,  0.0221;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
ProcessCloud::~ProcessCloud(){
    ros::shutdown();
    ros::waitForShutdown();
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessCloud::transformToCameraFrame(PointCloud<PointT>::Ptr nuvem){
    // Rotacionar a nuvem para cair no frame da câmera (laser tem X para frente, câmera deve ter
    // Z para frente e X para o lado direito)
    Matrix3f R;
    R = AngleAxisf(M_PI/2, Vector3f::UnitZ()) * AngleAxisf(-M_PI/2, Vector3f::UnitY());
    Matrix4f T = Matrix4f::Identity();     // Matriz de transformaçao homogenea
    T.block<3, 3>(0, 0) = R;                             // Adiciona a rotacao onde deve estar
    transformPointCloud(*nuvem, *nuvem, T);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessCloud::transformToCameraFrame(PointCloud<PointXYZ>::Ptr nuvem){
    // Rotacionar a nuvem para cair no frame da câmera (laser tem X para frente, câmera deve ter
    // Z para frente e X para o lado direito)
    Matrix3f R;
    R = AngleAxisf(M_PI/2, Vector3f::UnitZ()) * AngleAxisf(-M_PI/2, Vector3f::UnitY());
    Matrix4f T = Matrix4f::Identity();     // Matriz de transformaçao homogenea
    T.block<3, 3>(0, 0) = R;                             // Adiciona a rotacao onde deve estar
    transformPointCloud(*nuvem, *nuvem, T);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessCloud::colorCloudWithCalibratedImage(PointCloud<PointT>::Ptr cloud_in, Mat image, float scale){
    // Matriz da camera segundo escala da imagem
    MatrixXf P(3, 4);
    if(scale == 1)
        P = K1*Rt1;
    else if(scale == 4)
        P = K4*Rt4;

#pragma omp parallel for
    for(size_t i = 0; i < cloud_in->size(); i++){
        // Pegar ponto em coordenadas homogeneas
        MatrixXf X_(4, 1);
        X_ << cloud_in->points[i].x,
              cloud_in->points[i].y,
              cloud_in->points[i].z,
                        1          ;
        MatrixXf X(3, 1);
        X = P*X_;
        if(X(2, 0) > 0){
            X = X/X(2, 0);
            // Adicionando ponto na imagem se for o caso de projetado corretamente
            if(floor(X(0,0)) > 0 && floor(X(0,0)) < image.cols && floor(X(1,0)) > 0 && floor(X(1,0)) < image.rows){
                cv::Vec3b cor = image.at<Vec3b>(Point(X(0,0), X(1,0)));
                PointT point = cloud_in->points[i];
                point.b = cor.val[0]; point.g = cor.val[1]; point.r = cor.val[2];
                cloud_in->points[i] = point;
            }
        }
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessCloud::saveCloud(PointCloud<PointT>::Ptr nuvem, std::string nome){
    std::string nome_nuvem = pasta + nome + ".ply";
    savePLYFileBinary<PointT>(nome_nuvem, *nuvem);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessCloud::saveCloud(PointCloud<PointXYZ>::Ptr nuvem, std::string nome){
    std::string nome_nuvem = pasta + nome + ".ply";
    savePLYFileBinary<PointXYZ>(nome_nuvem, *nuvem);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
Matrix3f ProcessCloud::euler2matrix(float r, float p, float y){
    // Ja recebe os angulos aqui em radianos
    Matrix3f matrix;
    matrix = AngleAxisf(y, Vector3f::UnitY()) * AngleAxisf(r, Vector3f::UnitZ()) * AngleAxisf(p, Vector3f::UnitX());

    return matrix;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessCloud::transformCloudAndCamServoAngles(PointCloud<PointT>::Ptr cloud, float pan, float tilt, Vector3f &C, Quaternion<float> &q){
    /// Cria matriz de rotacao de acordo com angulos de pan e tilt
    // Angulos chegam em DEGREES - passar para RAD aqui
    // Pan - Yaw em torno de Y, negativo; Tilt - pitch em torno de X, negativo
    // Considerar primeiro o tilt, depois deslocar em pan
    pan = DEG2RAD(pan); tilt = DEG2RAD(tilt);

    Matrix3f Rt = euler2matrix(0, -tilt, 0);
    Matrix4f Tt = Matrix4f::Identity();
    Tt.block<3,3>(0, 0) = Rt;
    transformPointCloud(*cloud, *cloud, Tt);

    // Avanca a frente no eixo Z desde o eixo de rotacao
    Matrix3f Rp = euler2matrix(0, 0, -pan);
    Vector3f tp(0, 0, 0.056);
    tp = Rp*tp;
    // Finaliza a matriz homogenea e transformar a nuvem
    Matrix4f Tp = Matrix4f::Identity();
    Tp.block<3,3>(0, 0) = Rp;
    Tp.block<3,1>(0, 3) = tp;
    transformPointCloud(*cloud, *cloud, Tp);

    // Transformada final - nuvem
    Matrix4f T = Matrix4f::Identity();
    T = Tp*Tt;

    // Calculo do centro da camera - desenho do solid
    float bol = 0.056, blc = 0.0438, bac = 0.01;
    float x, y, z;
    // Primeiro braco
    z =  bol*cos(pan);
    x = -bol*sin(pan);
    // Segundo braco
    y = -blc*cos(tilt);
    z += blc*sin(tilt)*  cos(pan) ;
    x += blc*sin(tilt)*(-sin(pan));
    // Terceiro braco
    y += bac*sin(tilt);
    z += bac*cos(tilt)*  cos(pan) ;
    x += bac*cos(tilt)*(-sin(pan));
    // Inserindo offset do nivel horizontal
    y += blc;
    C << x, y, z;

    // Quaternion da camera
    Matrix3f R = T.block<3,3>(0, 0);
    Quaternion<float> q_temp(R.inverse());
    q = q_temp;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessCloud::setFolderName(string name){
    pasta = name;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
string ProcessCloud::getFolderName(){
    return pasta;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessCloud::cleanMisreadPoints(PointCloud<PointXYZ>::Ptr cloud){
    // Objeto para extrair indices com coordenadas na origem - ruido
    ExtractIndices<PointXYZ> extract;
    PointIndices::Ptr indices (new PointIndices);
    // Varrer a nuvem atras de pontos na origem
    for(size_t i=0; i<cloud->size(); i++){
        // Tambem remover pontos muito proximos, que nao fazem sentido
        float d = sqrt(pow(cloud->points[i].x, 2) + pow(cloud->points[i].y, 2) + pow(cloud->points[i].z, 2));
        if(((*cloud)[i].x == 0 && (*cloud)[i].y == 0 && (*cloud)[i].z == 0) || d <= 1 )
            indices->indices.push_back(i);
    }
    // Extrair fora esses pontos
    extract.setInputCloud(cloud);
    extract.setNegative(true);
    extract.setIndices(indices);
    extract.filter(*cloud);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
Vector2f ProcessCloud::getFocuses(int scale){
    Vector2f f;
    if(scale == 1){
        f(0) = K1(0, 0); f(1) = K1(1, 1);
    } else {
        f(0) = K4(0, 0); f(1) = K4(1, 1);
    }

    return f;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
Vector3f ProcessCloud::gettCam(){
  return Rt1.block<3,1>(0, 3);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessCloud::compileFinalSFM(vector<string> linhas){
  // Se ja existe o arquivo, deletar para sobreescrever
  struct stat buffer;
  string nome_sfm_final = pasta + "cameras.sfm";
  if(!stat(nome_sfm_final.c_str(), &buffer)){
    if(remove(nome_sfm_final.c_str()) == 0)
      ROS_INFO("Deletamos SFM anterior.");
    else
      ROS_ERROR("SFM final anterior nao foi deletado.");
  }
  // Anota num arquivo a partir do nome vindo
  ofstream sfm(nome_sfm_final);
  if(sfm.is_open()){

    sfm << std::to_string(linhas.size())+"\n\n"; // Quantas imagens
    for(int i=0; i < linhas.size(); i++)
      sfm << linhas[i]; // Imagem com detalhes de camera

  } // fim do if is open
  sfm.close(); // Fechar para nao ter erro
}
/////////////////////////////////////////////////////////////////////////////////////////////////
std::string ProcessCloud::escreve_linha_sfm(string nome, Matrix3f r, Vector3f t){
  string linha = pasta+nome;
  // Adicionar matriz de rotacao linha 1
  linha = linha + " " + std::to_string(r(0, 0)) + " " + std::to_string(r(0, 1)) + " " + std::to_string(r(0, 2));
  // Adicionar matriz de rotacao linha 2
  linha = linha + " " + std::to_string(r(1, 0)) + " " + std::to_string(r(1, 1)) + " " + std::to_string(r(1, 2));
  // Adicionar matriz de rotacao linha 3
  linha = linha + " " + std::to_string(r(2, 0)) + " " + std::to_string(r(2, 1)) + " " + std::to_string(r(2, 2));
  // Adicionar vetor de translacao
  linha = linha + " " + std::to_string(t(0)) + " " + std::to_string(t(1)) + " " + std::to_string(t(2));
  // Adicionar foco x e y como na matriz da camera em resolucao HD
  linha = linha + " " + std::to_string(K1(0, 0)) + " " + std::to_string(K1(1, 1));
  // Adicionar centro optico em x e y como na matriz da camera em resolucao HD
  linha = linha + " " + std::to_string(K1(0, 2)) + " " + std::to_string(K1(1, 2));
  // Adicionando quebra de linha - padrao do MART
  linha = linha + "\n";
  // Muda as virgulas por pontos no arquivo, caso existam
  std::replace(linha.begin(), linha.end(), ',', '.');
  return linha;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessCloud::blueprint(PointCloud<PointT>::Ptr cloud_in, float sa, float sr, Mat &bp){
  /// Separando a nuvem em clusters perpendiculares ao eixo y - y negativo para cima
  ///
  PointCloud<PointT>::Ptr cloud (new PointCloud<PointT>);
  *cloud = *cloud_in;
  // Filtrando a altura que vai entrar na roda
  float metros_altura_acima_pepo = 1; // quantos metros acima do PEPO para fazer a nuvem
  PassThrough<PointT> pass;
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-metros_altura_acima_pepo, 100); // Negativo de tudo no eixo Y
  pass.setNegative(false);
  pass.setInputCloud(cloud);
  pass.filter(*cloud);
  // Filtrando pontos fora da area de interesse
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-sa/2, sa/2); // Quadrado em Z
  pass.setInputCloud(cloud);
  pass.filter(*cloud);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(-sa/2, sa/2); // Quadrado em Z
  pass.setInputCloud(cloud);
  pass.filter(*cloud);
  int w = sa/sr, h = sa/sr;
  vector< vector<PointT> > supervoxels(w);
  for(int i=0; i<supervoxels.size(); i++) supervoxels[i].resize(h);
  // Preenchendo o os supervoxels com um ponto em altura inicial para a comparacao
  PointT pini;
  pini.y = 100;
#pragma omp parallel for
  for(int u=0; u<supervoxels.size(); u++){
    for(int v=0; v<supervoxels[u].size(); v++)
      supervoxels[u][v] = pini;
  }
  // Destinando cada ponto da nuvem original para o seu local no vetor de vetores segundo dimensoes, se for mais alto que o anterior
  for(size_t i=0; i<cloud->size(); i++){
    int u = abs(cloud->points[i].x - (-sa/2))/sa * w, v = abs(cloud->points[i].z - (-sa/2))/sa * h;
    if(u >= w) u = w - 1;
    if(v >= h) v = h - 1;
    if(cloud->points[i].y < supervoxels[u][v].y)
      supervoxels[u][v] = cloud->points[i];
  }
  cloud->clear();

  /// Colorindo a imagem final de acordo com a resolucao da separacao da nuvem
  ///
  // Iniciar a imagem com a quantidade de pixels de acordo com numero de supervoxels
  bp = Mat::zeros(cv::Size(w, h), CV_8UC3);
  // Processando em paralelo, procurar ponto mais alto de cada supervoxel
#pragma omp parallel for
  for(int u=0; u<supervoxels.size(); u++){
    for(int v=0; v<supervoxels[u].size(); v++){
      PointT p = supervoxels[u][v];
      Vec3b cor;
      // Atribuir cor do ponto mais alto aquele lugar da foto
      cor.val[0] = p.b; cor.val[1] = p.g; cor.val[2] = p.r;
      bp.at<Vec3b>(h-1-v, u) = cor;
    }
  }

  /// Salvando
  ///
  imwrite(pasta+"planta_baixa.png", bp);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessCloud::getVirtualImage(PointCloud<PointXYZ>::Ptr cloud_in, Mat image, Mat &iv, float scale){
    // Matriz da camera segundo escala da imagem
    MatrixXf P(3, 4);
    if(scale == 1)
        P = K1*Rt1;
    else if(scale == 4)
        P = K4*Rt4;

#pragma omp parallel for
    for(size_t i = 0; i < cloud_in->size(); i++){
        // Pegar ponto em coordenadas homogeneas
        MatrixXf X_(4, 1);
        X_ << cloud_in->points[i].x,
              cloud_in->points[i].y,
              cloud_in->points[i].z,
                        1          ;
        MatrixXf X(3, 1);
        X = P*X_;
        if(X(2, 0) > 0){
            X = X/X(2, 0);
            // Adicionando ponto na imagem virtual se for o caso de projetado corretamente
            if(floor(X(0,0)) > 0 && floor(X(0,0)) < image.cols && floor(X(1,0)) > 0 && floor(X(1,0)) < image.rows){
                cv::Vec3b cor = image.at<Vec3b>(Point(X(0,0), X(1,0)));
                iv.at<Vec3b>(Point(X(0,0), X(1,0))) = cor;
            }
        }
    }
}

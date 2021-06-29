#include "../include/processcloud.h"

/////////////////////////////////////////////////////////////////////////////////////////////////
ProcessCloud::ProcessCloud(string _pasta):pasta(_pasta){
    //    // Matriz intrinseca com imagem em Full HD
    //    K1 << 1427.1  ,   -0.063, 987.9,
    //             0.041, 1449.4  , 579.4,
    //             0    ,    0    ,   1  ;
    // Lendo arquivo onde esta a calibracao
    char* home;
    home = getenv("HOME");
    ifstream calib_file(string(home)+"/calib_cam_laser.txt");
    string params_line;
    vector<float> params;
    if(calib_file.is_open()){
        while(getline(calib_file, params_line)){
            istringstream iss(params_line);
            for(string s; iss >> s; )
                params.push_back(stof(s));
        }
    }
    calib_file.close();
    // Matriz intrinseca com imagem em HD
    K1 << params[0], params[1], params[2],
          params[3], params[4], params[5],
          params[6], params[7], params[8];
//    // Matriz intrinseca com imagem em HD
//    K1 << 978.34 ,  -0.013, 654.28,
//            0.054, 958.48 , 367.44,
//            0    ,   0    ,   1   ;
    // Matriz intrinseca com imagem em resolucao simplificada 480x270
    K4 << 241.60 ,  -0.025, 162.19,
            0.073, 240.91 ,  92.41,
            0    ,   0    ,   1   ;
    // Matriz extrinseca com imagem em HD
    Rt1.resize(3, 4);
    Rt1 << 1, 0, 0, params[ 9],
           0, 1, 0, params[10],
           0, 0, 1, params[11];
//    Rt1 << 1, 0, 0, -0.011,
//           0, 1, 0,  0.029,
//           0, 0, 1,  0.027;
    // Matriz extrinseca com imagem em resolucao simplificada 480x270
    Rt4.resize(3, 4);
    Rt4 << 1, 0, 0,  0.035,
           0, 1, 0,  0.013,
           0, 0, 1, -0.041;
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
    matrix =  AngleAxisf(y, Vector3f::UnitY()) * AngleAxisf(r, Vector3f::UnitZ()) * AngleAxisf(p, Vector3f::UnitX());

    return matrix;
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
        if(((*cloud)[i].x == 0 && (*cloud)[i].y == 0 && (*cloud)[i].z == 0) || d <= 2 )
            indices->indices.push_back(i);
    }
    // Extrair fora esses pontos
    extract.setInputCloud(cloud);
    extract.setNegative(true);
    extract.setIndices(indices);
    extract.filter(*cloud);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessCloud::cleanNotColoredPoints(PointCloud<PointT>::Ptr cloud){
    // Objeto para extrair indices com coordenadas na origem - ruido
    ExtractIndices<PointT> extract;
    PointIndices::Ptr indices (new PointIndices);
    // Varrer a nuvem atras de pontos na origem
    for(size_t i=0; i<cloud->size(); i++){
        // Tambem remover pontos muito proximos, que nao fazem sentido
        bool colored = (cloud->points[i].r != 200 && cloud->points[i].g != 200 && cloud->points[i].b != 200);
        if(colored)
            indices->indices.push_back(i);
    }
    // Seguem esses pontos coloridos
    extract.setInputCloud(cloud);
    extract.setNegative(false);
    extract.setIndices(indices);
    extract.filter(*cloud);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
Vector2f ProcessCloud::getFocuses(int scale){
    Vector2f f;
    if(scale == 1)
        f(0) = K1(0, 0); f(1) = K1(1, 1);

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
                iv.at<Vec3b>( Point(int(X(0,0)/4), int(X(1,0)/4)) ) = cor;
            }
        }
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////

#include "../include/processcloud.h"

/////////////////////////////////////////////////////////////////////////////////////////////////
ProcessCloud::ProcessCloud(string _pasta):pasta(_pasta){

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
    // Matriz intrinseca e extrinseca
    Matrix3f K;
    K << 1525.4  /scale,   -0.024/scale, 961/scale,
           -0.001/scale, 1515.9  /scale, 541/scale,
            0          ,    0          ,   1      ;
    MatrixXf Rt(3, 4);                               // Otimizacao com Matlab
    Rt << 1, 0, 0,  0.053,
          0, 1, 0,  0.116,
          0, 0, 1,  0.121;
    MatrixXf P(3, 4);
    P = K*Rt;
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
void ProcessCloud::saveImage(cv::Mat img, string nome){
    std::string final = pasta + nome + ".png";
    //vector<int> params;
    //params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    //params.push_back(9);
    cv::imwrite(final, img);//, params);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
Matrix3f ProcessCloud::euler2matrix(float r, float p, float y){
    // Ja recebe os angulos aqui em radianos
    Matrix3f matrix;
    matrix = AngleAxisf(y, Vector3f::UnitY()) * AngleAxisf(p, Vector3f::UnitX());

    return matrix;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
std::string ProcessCloud::escreve_linha_imagem(float foco, std::string nome, Vector3f C, Quaternion<float> q){
    std::string linha = pasta+nome+".png";
    // Adicionar foco
    linha = linha + " " + std::to_string(foco);
    // Adicionar quaternion
    linha = linha + " " + std::to_string(q.w()) + " " + std::to_string(q.x()) + " " + std::to_string(q.y()) + " " + std::to_string(q.z());
    // Adicionar centro da camera
    linha = linha + " " + std::to_string(C(0)) + " " + std::to_string(C(1)) + " " + std::to_string(C(2));
    // Adicionar distorcao radial (crendo 0) e 0 final
    linha = linha + " 0 0\n"; // IMPORTANTE pular linha aqui, o MeshRecon precisa disso no MART
    // Muda as virgulas por pontos no arquivo
    std::replace(linha.begin(), linha.end(), ',', '.');
    return linha;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessCloud::compileFinalNVM(vector<string> linhas){
    // Anota num arquivo a partir do nome vindo
    ofstream nvm(pasta+"cameras.nvm");
    if(nvm.is_open()){

        nvm << "NVM_V3\n\n";
        nvm << std::to_string(linhas.size())+"\n"; // Quantas imagens, sempre uma aqui
        for(int i=0; i < linhas.size(); i++)
            nvm << linhas[i]; // Imagem com detalhes de camera

    } // fim do if is open
    nvm << "\n";
    nvm.close(); // Fechar para nao ter erro
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
        if((*cloud)[i].x == 0 && (*cloud)[i].y == 0 && (*cloud)[i].z == 0)
            indices->indices.push_back(i);
    }
    // Extrair fora esses pontos
    extract.setInputCloud(cloud);
    extract.setNegative(true);
    extract.setIndices(indices);
    extract.filter(*cloud);
}
/////////////////////////////////////////////////////////////////////////////////////////////////

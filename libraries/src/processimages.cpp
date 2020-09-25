#include "../include/processimages.h"

/////////////////////////////////////////////////////////////////////////////////////////////////
ProcessImages::ProcessImages(string _pasta):pasta(_pasta){

}
/////////////////////////////////////////////////////////////////////////////////////////////////
ProcessImages::~ProcessImages(){
    ros::shutdown();
    ros::waitForShutdown();
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessImages::saveImage(Mat img, string nome){
    string final = pasta + nome + ".png";
    imwrite(final, img);//, params);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
string ProcessImages::escreve_linha_imagem_NVM(float foco, string nome, Vector3f C, Quaternion<float> q){
    string linha = pasta+nome+".png";
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
void ProcessImages::compileFinalNVM(vector<string> linhas){
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
void ProcessImages::setFolderName(string name){
    pasta = name;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
string ProcessImages::getFolderName(){
    return pasta;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessImages::estimateRaw360(vector<Quaternion<float>> qs, Vector2f fs){
    ////////// Inicia a esfera
    ///
    // Supoe a esfera com resolucao em graus de tal forma - resolucao da imagem final
    float R = 5; // Raio da esfera [m]
    // Angulos para lat e lon, 360 de range para cada, resolucao a definir no step_deg
    float step_deg = 0.1; // [DEGREES]
    int raios_360 = int(360.0 / step_deg), raios_180 = raios_360 / 2.0; // Quantos raios sairao do centro para formar 360 e 180 graus de um circulo 2D

    ////////// Ler cada imagem e processar no seu lugar da esfera
    ///
    Mat im360 = Mat::zeros(Size(raios_360, raios_180), CV_8UC3); // Imagem 360 ao final de todas as fotos passadas
    #pragma omp parallel for
    for (int i = 0; i < qs.size(); i++)
    {
        // Ler a imagem a ser usada
        string nome_imagem;
        if((i + 1) < 10)
            nome_imagem = "imagem_00"+std::to_string(i+1)+".png";
        else if((i + 1) < 100)
            nome_imagem = "imagem_0" +std::to_string(i+1)+".png";
        else
            nome_imagem = "imagem_"  +std::to_string(i+1)+".png";
        Mat image = imread(pasta+nome_imagem);
        if (image.cols < 3)
            cout << "Imagem nao foi encontrada, problema ..." << endl;

        // Calcular a vista da camera pelo Rt inverso - rotacionar para o nosso mundo, com Z para cima
        Matrix4f T = Matrix4f::Identity();
        T.block<3,3>(0, 0) = qs[i].matrix();
        // Definir o foco em dimensoes fisicas do frustrum
        float F = R;
        double minX, minY, maxX, maxY;
        maxX = F * (float(image.cols) / (2.0*fs[0]));
        minX = -maxX;
        maxY = F * (float(image.rows) / (2.0*fs[1]));
        minY = -maxY;
        // Calcular os 4 pontos do frustrum
        /*
                                                                    origin of the camera = p1
                                                                    p2--------p3
                                                                    |          |
                                                                    |  pCenter |<--- Looking from p1 to pCenter
                                                                    |          |
                                                                    p5--------p4
        */
        Vector4f p, p1, p2, p3, p4, p5, pCenter;
        p << 0, 0, 0, 1;
        p1 = T * p;
        p << minX, minY, F, 1;
        p2 = T * p;
        p << maxX, minY, F, 1;
        p3 = T * p;
        p << maxX, maxY, F, 1;
        p4 = T * p;
        p << minX, maxY, F, 1;
        p5 = T * p;
        p << 0, 0, F, 1;
        pCenter = T * p;
        // Fazer tudo aqui nessa nova funcao, ja devolver a imagem esferica inclusive nesse ponto
        this->doTheThing(step_deg, p2.block<3, 1>(0, 0), p4.block<3, 1>(0, 0), p5.block<3, 1>(0, 0), image, im360);
    }

    ////////// Salvar imagem final na pasta
    ///
    imwrite(pasta + "panoramica.png", im360);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessImages::doTheThing(float sd, Vector3f p2, Vector3f p4, Vector3f p5, Mat im, Mat &im360) {
    // A partir de frustrum, calcular a posicao de cada pixel da imagem fonte em XYZ, assim como quando criavamos o plano do frustrum
    Vector3f hor_step, ver_step; // Steps pra se andar de acordo com a resolucao da imagem
    hor_step = (p4 - p5) / float(im.cols);
    ver_step = (p2 - p5) / float(im.rows);
#pragma omp parallel for
    for (int i = 0; i < im.rows; i++) { // Vai criando o frustrum a partir dos cantos da imagem
        for (int j = 0; j < im.cols; j++) {
            Vector3f ponto;
            ponto = p5 + hor_step * j + ver_step * i;

            // Calcular latitude e longitude da esfera de volta a partir de XYZ
            float lat =  RAD2DEG(acos(ponto[1] / ponto.norm()));
            float lon = -RAD2DEG(atan2(ponto[2], ponto[0]));
            lon = (lon < 0) ? lon += 360.0 : lon; // Ajustar regiao do angulo negativo, mantendo o 0 no centro da imagem

            // Pelas coordenadas, estimar posicao do pixel que deve sair na 360 final e pintar - da forma como criamos a esfera
            int u = int(lon / sd);
            u = (u >= im360.cols) ? im360.cols - 1 : u; // Nao deixar passar do limite de colunas por seguranca
            u = (u < 0) ? 0 : u;
            int v = im360.rows - 1 - int(lat / sd);
            v = (v >= im360.rows) ? im360.rows - 1 : v; // Nao deixar passar do limite de linhas por seguranca
            v = (v < 0) ? 0 : v;
            // Pintar a imagem final com as cores encontradas
            im360.at<Vec3b>(Point(u, v)) = im.at<Vec3b>(Point(j, im.rows - 1 - i));
        }
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////

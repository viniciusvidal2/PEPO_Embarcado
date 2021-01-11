#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/videoio.hpp"
#include <opencv2/calib3d.hpp>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "blur");
    ros::NodeHandle nh;

    for(int i=38; i<48; i++){

        // Ler imagem
        Mat im;
        if(i < 10)
            im = imread("/home/vinicius/Desktop/SANTOS_DUMONT_2/sala/scan3/imagem_00"+std::to_string(i)+".png", IMREAD_GRAYSCALE);
        else
            im = imread("/home/vinicius/Desktop/SANTOS_DUMONT_2/sala/scan3/imagem_0"+std::to_string(i)+".png", IMREAD_GRAYSCALE);

        // Calcular filtro gaussiando e variancia
        Mat lap, lap_8b;
        Laplacian(im, lap, CV_16SC1, 3, 1, 0, BORDER_DEFAULT);
        convertScaleAbs( lap, lap_8b );

        Scalar m, s;
        meanStdDev(lap, m, s, Mat());
        cout << "\nVariance: " << (s*s)[0] << endl;

        // Mostrar a imagem com o numero em texto
        putText(im, "Variance: "+std::to_string((s*s)[0]), Point(20, 50), FONT_HERSHEY_COMPLEX, 1, Scalar(0, 255, 0));
        imshow("Final", im);
        waitKey(0);

    }
    return 0;
}

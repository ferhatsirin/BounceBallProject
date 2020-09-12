#ifndef IMAGEPROCESSING_H
#define IMAGEPROCESSING_H

#include <opencv2/core.hpp>
#include "mainwindow.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <QTimer>
#include <string>
#include <QtSerialPort/QSerialPort>
#include "sender.h"

using namespace std;
using namespace cv;

class ImageProcessing : public QObject
{
    Q_OBJECT
public:

    ImageProcessing(MainWindow* ui);
    ~ImageProcessing();
    Vec3f findBall(Mat& img);
    float findDistance(Vec3f& pos, float KNOWN_AREA, float FOCAL_LENGTH);
    float calculateFocalLength(Vec3f& pos, float INITIAL_DISTANCE, float KNOWN_AREA);
    void setMotorDegree(int degree);
    void sendPositionData(int x,int y);


private slots:
    void setCamera();
    void processImage();
    void setServer();
    void drawCircleCommand();
    void setPort();
    void sendPosition();
    void bounceBall();
    void minH_value(int value);
    void minS_value(int value);
    void minV_value(int value);
    void maxH_value(int value);
    void maxS_value(int value);
    void maxV_value(int value);
    void setShowRange();

private :

    const float KNOWN_AREA = 12.56637;
    const float INITIAL_DISTANCE = 20;
    const float FOCAL_LENGTH = 14442.1;
    const string outputFileName ="data.xml";

    Mat img;
    Size imgSize;
    Mat hsvImg, range,blured;

    Sender sender;

    QSerialPort port;

    VideoCapture cap;
    QTimer * timer;
    MainWindow* ui;
    QLabel* imgLabel;

    bool showRange;

    int lowH ; // Set Hue
    int highH ;

    int lowS ; // Set Saturation
    int highS ;

    int lowV ; // Set Value
    int highV ;

};

#endif // IMAGEPROCESSING_H

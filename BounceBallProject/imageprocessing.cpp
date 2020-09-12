#include "imageprocessing.h"

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <QComboBox>
#include <QCameraInfo>

#include <QtSerialPort/QSerialPortInfo>


using namespace cv;
using namespace std;

Q_DECLARE_METATYPE(QCameraInfo)
Q_DECLARE_METATYPE(QSerialPortInfo)

ImageProcessing::ImageProcessing(MainWindow* ui) : sender(ui)
{

    this->ui = ui;
    imgLabel = ui->getImageLabel();

    QComboBox* list = ui->getCameraBox();

    foreach (const QCameraInfo& cameraInfo, QCameraInfo::availableCameras())
    {
        list->addItem(cameraInfo.deviceName(), QVariant::fromValue<QCameraInfo>(cameraInfo));

    }

    QComboBox* portList = ui->getPortList();

    foreach (const QSerialPortInfo& inf , QSerialPortInfo::availablePorts())
    {
        portList->addItem(inf.portName(),QVariant::fromValue<QSerialPortInfo>(inf));
    }

    lowH = 108;
    lowS =44;
    lowV = 41;

    highH = 255;
    highS = 255;
    highV = 255;

    ui->getMinH_slider()->setSliderPosition(lowH);
    ui->getMinS_slider()->setSliderPosition(lowS);
    ui->getMinV_slider()->setSliderPosition(lowV);
    ui->getMaxH_slider()->setSliderPosition(highH);
    ui->getMaxS_slider()->setSliderPosition(highS);
    ui->getMaxV_slider()->setSliderPosition(highV);
    minH_value(lowH);
    minS_value(lowS);
    minV_value(lowV);
    maxH_value(highH);
    maxS_value(highS);
    maxV_value(highV);

    showRange =false;

    timer = new QTimer(this);

    connect(ui->getCameraButton(),SIGNAL(clicked()),this, SLOT(setCamera()));
    connect(ui->getConnectButton(),SIGNAL(clicked()),this,SLOT(setServer()));
    connect(ui->getPortButton(),SIGNAL(clicked()),this,SLOT(setPort()));
    connect(ui->getPosButton(),SIGNAL(clicked()),this,SLOT(sendPosition()));
    connect(ui->getBounceButton(),SIGNAL(clicked()),this,SLOT(bounceBall()));
    connect(ui->getShowRangeButton(),SIGNAL(clicked()),this,SLOT(setShowRange()));
    connect(ui->getDrawCircleButton(),SIGNAL(clicked()),this,SLOT(drawCircleCommand()));
    connect(ui->getMinH_slider(),SIGNAL(valueChanged(int)),this,SLOT(minH_value(int)));
    connect(ui->getMinS_slider(),SIGNAL(valueChanged(int)),this,SLOT(minS_value(int)));
    connect(ui->getMinV_slider(),SIGNAL(valueChanged(int)),this,SLOT(minV_value(int)));
    connect(ui->getMaxH_slider(),SIGNAL(valueChanged(int)),this,SLOT(maxH_value(int)));
    connect(ui->getMaxS_slider(),SIGNAL(valueChanged(int)),this,SLOT(maxS_value(int)));
    connect(ui->getMaxV_slider(),SIGNAL(valueChanged(int)),this,SLOT(maxV_value(int)));
    connect(timer, SIGNAL(timeout()),this, SLOT(processImage()));

    timer->start(30);

}

ImageProcessing::~ImageProcessing(){

    if(cap.isOpened())
        cap.release();

    if(port.isOpen()){
        port.close();
    }

    sender.closeSocket();
}

void ImageProcessing::minH_value(int value){
    lowH =value;
    ui->setMinH_label(value);
}
void ImageProcessing::minS_value(int value){
    lowS =value;
    ui->setMinS_label(value);
}
void ImageProcessing::minV_value(int value){
    lowV =value;
    ui->setMinV_label(value);
}
void ImageProcessing::maxH_value(int value){
    highH =value;
    ui->setMaxH_label(value);
}
void ImageProcessing::maxS_value(int value){
    highS =value;
    ui->setMaxS_label(value);
}
void ImageProcessing::maxV_value(int value){
    highV =value;
    ui->setMaxV_label(value);
}

void ImageProcessing::setShowRange(){

    if(showRange){
        showRange =false;
        ui->getShowRangeButton()->setText("Show\nRange");
    }else{
        showRange = true;
        ui->getShowRangeButton()->setText("Close\nRange");
    }
}


void ImageProcessing::setCamera()
{

    string str =ui->getCameraBox()->currentText().toStdString();

    if(cap.isOpened()){
        cap.release();
    }

    int id;

    if(str != "" && str.find("/dev/video") != string::npos){

        sscanf(str.c_str(),"/dev/video%d",&id);
    }else{
        id = ui->getCameraBox()->currentIndex();
    }

    if(cap.open(id)){

    }

}

void ImageProcessing::setServer(){

    sender.listenServer();

}

void ImageProcessing::setPort(){

    QSerialPortInfo inf = ui->getPortList()->currentData().value<QSerialPortInfo>();

    if(port.isOpen())
        port.close();

    port.setPort(inf);

    if(port.open(QIODevice::ReadWrite)){
        ui->setConnectionPortText("Connected");

    }else{
        ui->setConnectionPortText("Disconnected");
    }
}


void ImageProcessing::setMotorDegree(int degree){

    if(port.isOpen()){

        char data[5];

        data[0]= 10;

        data[1] = degree & 0xFF;
        data[2] = (degree >> 8) & 0xFF;

        port.write(data,5);
    }
}

void ImageProcessing::drawCircleCommand(){

    if(port.isOpen()){
        sendPositionData(200,100);
        waitKey(6000);
        sendPositionData(350,100);
        waitKey(6000);
        sendPositionData(400,300);
        waitKey(6000);
        sendPositionData(200,300);
    }
}


void ImageProcessing::sendPositionData(int x,int y){

    if(port.isOpen()){
        char data[5];

        data[0] =100;

        data[1] = x & 0xFF;
        data[2] = (x >> 8) & 0xFF;

        data[3] = y & 0xFF;
        data[4] = (y >> 8) & 0xFF;

        port.write(data,5);

    }
}

void ImageProcessing::sendPosition(){

    if(port.isOpen()){

        string x = ui->getPosX();
        string y =ui->getPosY();

        if(x != "" && y != ""){
            int posX, posY;

            sscanf(x.c_str(),"%d",&posX);
            sscanf(y.c_str(),"%d",&posY);

            sendPositionData(posX,posY);

        }
    }
}

void ImageProcessing::bounceBall(){
    if(port.isOpen()){
        char data[5];

        data[0] =150;

        port.write(data,5);

    }

}

void ImageProcessing::processImage()
{
    if(cap.isOpened()){

        Vec3f pos;
        float distance;
        char str[15];
        char data[5];

        cap>>img;

        pos = findBall(img);
        distance = findDistance(pos, KNOWN_AREA, FOCAL_LENGTH);

        if(port.isOpen()){  // send current position

            if(pos[0] == 0 && pos[1] == 0){
                setMotorDegree(5);
            }else{
                data[0] = 50;

                data[1] = (int)pos[0] & 0xFF;
                data[2] = ((int)pos[0] >> 8) & 0xFF;

                data[3] = (int)pos[1] & 0xFF;
                data[4] = ((int)pos[1] >> 8) & 0xFF;
                port.write(data,5);

            }
        }

        if( 0 < pos[0] && 0 < pos[1] && distance < 100){
            sprintf(str, "%.2f cm", distance);
            putText(img, str, Point(pos[0]+pos[2], pos[1]), FONT_HERSHEY_SIMPLEX, 1.3, CV_RGB(0, 255, 0), 2);
        }

        if(sender.isOpen()){
            sender.sendData(pos[0],pos[1]);
        }

        if(!showRange){
            cvtColor(img, img, COLOR_BGR2RGB);

            // image is created according to Mat dimensions
            QImage image(img.data, img.size().width, img.size().height, img.step, QImage::Format_RGB888);

            // Display the QImage in a QLabel
            imgLabel->setPixmap(QPixmap::fromImage(image));

        }else{
            cvtColor(range,range,COLOR_BGR2RGB);

            QImage image(range.data,range.size().width,range.size().height,range.step,QImage::Format_RGB888);
            imgLabel->setPixmap(QPixmap::fromImage(image));
        }

        ui->setPosition((int)pos[0],(int)pos[1]);
        if(distance < 100)
            ui->setDistance((int)distance);
        else
            ui->setDistance(0);
    }
}


float ImageProcessing::findDistance(Vec3f& pos, float KNOWN_AREA, float FOCAL_LENGTH) {

    float radius = pos[2];
    float area = M_PI * pow(radius, 2.0);

    float distance = KNOWN_AREA * FOCAL_LENGTH / area;

    return distance;

}

Vec3f ImageProcessing::findBall(Mat& img) {

    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));

    vector<vector < Point>> contours;
    vector<cv::Vec4i> hierarchy;
    float radius;
    Point2f circle;

    cvtColor(img, hsvImg, COLOR_BGR2HSV);

    inRange(hsvImg, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), range);

    GaussianBlur(range, blured, Size(0, 0), 3);
    addWeighted(range, 1.5, blured, -0.5, 0, range);
    erode(range, range, element);
    dilate(range, range, element);

    // fill circles vector with all circles in processed image
    findContours(range, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE); // algorithm for detecting circles

    if (0 < contours.size()) {

        vector<Point> c = *std::max_element(contours.begin(),
                                            contours.end(),
                                            [](std::vector<cv::Point> const& lhs, std::vector<cv::Point> const& rhs) {
                return contourArea(lhs, false) < contourArea(rhs, false);
    });

        minEnclosingCircle(c, circle, radius);

        if(30 < radius && radius < 60  ){
            cv::circle(img, circle, (int) radius, Scalar(100, 200, 200), 5);

            return Vec3f(circle.x,circle.y,radius);
        }
        else
            return Vec3f(0,0,0);

    } else {

        return Vec3f(0, 0, 0);
    }
}

float ImageProcessing::calculateFocalLength(Vec3f& pos, float INITIAL_DISTANCE, float KNOWN_AREA) {
    float radius = pos[2];
    float area = M_PI * pow(radius, 2.0);

    float focal = INITIAL_DISTANCE * area / KNOWN_AREA;

    return focal;
}


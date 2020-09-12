#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QString>
#include <QCloseEvent>
#include <QComboBox>
#include <string>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);


}

QLabel* MainWindow::getImageLabel(){

    return ui->label;
}

QSlider* MainWindow::getMinH_slider(){
    return ui->minH_slider;
}

QSlider* MainWindow::getMinS_slider(){
    return ui->minS_slider;
}

QSlider* MainWindow::getMinV_slider(){
    return ui->minV_slider;
}

QSlider* MainWindow::getMaxH_slider(){
    return ui->maxH_slider;
}

QSlider* MainWindow::getMaxS_slider(){
    return ui->maxS_slider;
}

QSlider* MainWindow::getMaxV_slider(){
    return ui->maxV_slider;
}

void MainWindow::setMinH_label(int v){
    ui->minH_L->setText(QString::number(v));
}
void MainWindow::setMinS_label(int v){
    ui->minS_L->setText(QString::number(v));
}
void MainWindow::setMinV_label(int v){
    ui->minV_L->setText(QString::number(v));
}
void MainWindow::setMaxH_label(int v){
    ui->maxH_L->setText(QString::number(v));
}
void MainWindow::setMaxS_label(int v){
    ui->maxS_L->setText(QString::number(v));
}
void MainWindow::setMaxV_label(int v){
    ui->maxV_L->setText(QString::number(v));
}

QPushButton* MainWindow::getShowRangeButton(){
    return ui->showRange_btn;
}
void MainWindow::setPosition(int x,int y){

    ui->posText->setText(QString::number(x)+", "+QString::number(y));
}

void MainWindow::setDistance(int x){

    ui->distText->setText(QString::number(x));
}

QComboBox* MainWindow::getCameraBox(){

    return ui->cameraList;
}

string MainWindow::getPosX(){
    return ui->posX->text().toStdString();
}

string MainWindow::getPosY(){
    return ui->posY->text().toStdString();
}

QPushButton* MainWindow::getPosButton(){
    return ui->pos_btn;
}

QPushButton* MainWindow::getDrawCircleButton(){

    return ui->drawCircle_btn;
}

QComboBox* MainWindow::getPortList(){
    return ui->portList;
}

void MainWindow::setConnectionPortText(string str){
    ui->connectPort_txt->setText(QString::fromStdString(str));
}

QPushButton* MainWindow::getCameraButton(){
    return ui->camera_btn;
}

QPushButton* MainWindow::getPortButton(){
    return ui->port_btn;
}

QPushButton* MainWindow::getConnectButton(){
    return ui->connect_btn;
}

QPushButton* MainWindow::getBounceButton(){
    return ui->bounce_btn;
}

string MainWindow::getIPAddress(){
    return ui->ipText->text().toStdString();
}

void MainWindow::setConnectionText(string str){

    ui->connect_txt->setText(QString::fromStdString(str));
}

void MainWindow::closeEvent (QCloseEvent *event)
{
    event->accept();

}


MainWindow::~MainWindow()
{
    delete ui;
}

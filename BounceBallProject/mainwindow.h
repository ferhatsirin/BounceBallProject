#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QComboBox>
#include <QPushButton>
#include <string>

using namespace std;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    QComboBox* getCameraBox();
    QLabel* getImageLabel();
    QSlider* getMinH_slider();
    QSlider* getMinS_slider();
    QSlider* getMinV_slider();
    QSlider* getMaxH_slider();
    QSlider* getMaxS_slider();
    QSlider* getMaxV_slider();
    void setMinH_label(int v);
    void setMinS_label(int v);
    void setMinV_label(int v);
    void setMaxH_label(int v);
    void setMaxS_label(int v);
    void setMaxV_label(int v);
    QPushButton* getShowRangeButton();
    QPushButton* getCameraButton();
    void setPosition(int x,int y);
    QPushButton* getConnectButton();
    string getIPAddress();
    QComboBox* getPortList();
    QPushButton* getPortButton();
    QPushButton* getDrawCircleButton();
    QPushButton* getBounceButton();
    void setConnectionPortText(string str);
    string getPosX();
    string getPosY();
    QPushButton* getPosButton();
    void setConnectionText(string str);
    void setDistance(int x);
    void closeEvent (QCloseEvent *event);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H

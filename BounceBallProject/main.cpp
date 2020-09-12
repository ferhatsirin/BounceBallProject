#include "mainwindow.h"
#include <QApplication>
#include "imageprocessing.h"

using namespace std;

int main(int argc, char *argv[])
{

    QApplication a(argc, argv);

    MainWindow w;

    ImageProcessing process(&w);

    w.show();

    return a.exec();
}


#ifndef SENDER_H
#define SENDER_H


#include <iostream>
#include<QWidget>
#include<QTcpSocket>
#include<QDebug>
#include <QTcpServer>
#include "mainwindow.h"

class Sender: public QObject
{
    Q_OBJECT
public:
    Sender(MainWindow* ui);
    void sendData(int x, int y);
    void listenServer();
    void closeSocket();
    bool isOpen();

signals:

public slots:
    void connectDevice();
    void onSocketStateChanged(QAbstractSocket::SocketState socketState);

private:
    QTcpServer server;
    QTcpSocket *clientSocket;
    bool open;
    MainWindow* ui;

};


#endif // SENDER_H

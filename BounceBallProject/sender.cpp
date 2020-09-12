#include "sender.h"
#include <string>
#include <iostream>


using namespace std;

Sender::Sender(MainWindow* ui) {

    this->ui = ui;

    server.connect(&server, SIGNAL(newConnection()), this, SLOT(connectDevice()));

    open =false;

}

void Sender::listenServer(){
    server.listen(QHostAddress::AnyIPv4,8080);
    ui->setConnectionText("Listening");
}

void Sender::connectDevice(){

    clientSocket = server.nextPendingConnection();

    connect(clientSocket, SIGNAL(stateChanged(QAbstractSocket::SocketState)), this, SLOT(onSocketStateChanged(QAbstractSocket::SocketState)));

    open = true;
    ui->setConnectionText("Connected");

}

void Sender::onSocketStateChanged(QAbstractSocket::SocketState socketState)
{

    if (socketState == QAbstractSocket::UnconnectedState)
    {
        open = false;
        ui->setConnectionText("Disconnected");
    }
}


bool Sender::isOpen(){

    return open;
}

void Sender::closeSocket(){

    open =false;
    server.close();
}


void Sender::sendData(int x, int y){

    if(open){

        string str = to_string(x) + "," + to_string(y)+ "\0";

        clientSocket->write(str.c_str(),str.size());
    }
}

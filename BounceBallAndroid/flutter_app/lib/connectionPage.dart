import 'dart:convert';
import 'dart:io';

import 'package:flutter/cupertino.dart';
import 'package:flutter/material.dart';
import 'myHomePage.dart';

class ConnectionPage extends StatefulWidget {
  final String title;

  ConnectionPage({this.title});

  @override
  _ConnectionPageState createState() => _ConnectionPageState();
}

class _ConnectionPageState extends State<ConnectionPage> {
  final myControllerIp = TextEditingController();
  final myControllerPort = TextEditingController();
  Socket sock;

  @override
  void initState() {
    super.initState();

    myControllerIp.addListener(_printLatestValue);
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text(widget.title),
      ),
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.start,
          children: <Widget>[
            SizedBox(
              height: 50,
            ),
            Text(
              'Enter Ip Address And Port Number To Connect Simulation',
              style: TextStyle(
                color: Colors.blue,
                fontSize: 15,
              ),
            ),
            SizedBox(
              height: 50,
            ),
            TextField(
              controller: myControllerIp,
              decoration: new InputDecoration(
                labelText: "Enter Ip Address",
                fillColor: Colors.white,
                border: new OutlineInputBorder(
                  borderRadius: new BorderRadius.circular(25.0),
                  borderSide: new BorderSide(
                  ),
                ),
              ),
            ),
            SizedBox(
              height: 20,
            ),

            TextField(
              controller: myControllerPort,
              decoration: new InputDecoration(
                labelText: "Enter Port Number",
                fillColor: Colors.white,
                border: new OutlineInputBorder(
                  borderRadius: new BorderRadius.circular(25.0),
                  borderSide: new BorderSide(
                  ),
                ),
              ),
            ),
          ],
        ),
      ),
      floatingActionButton: FloatingActionButton(
        onPressed: _incrementCounter,
        tooltip: 'Increment',
        child: Icon(Icons.arrow_forward),
      ),
    );
  }

  Future<void> _incrementCounter() async {

    sock = await Socket.connect(myControllerIp.text,  int.tryParse(myControllerPort.text) ?? 8080);
    //sock = await Socket.connect("192.168.43.228", 8080);


    Navigator.push(
        context,
        MaterialPageRoute(
          builder: (context) => MyHomePage(title: 'Flutter Demo Home Page', socket: sock),
        ));
  }

  _printLatestValue() {
    print("Port text field: ${myControllerPort.text}");
    print("Ip text field: ${myControllerIp.text}");
  }
}

import 'dart:convert';
import 'dart:io';

import 'package:flutter/material.dart';
import 'package:flutter_app/circle.dart';

class MyHomePage extends StatefulWidget {
  MyHomePage({this.title, this.socket});

  final String title;
  final Socket socket;

  @override
  _MyHomePageState createState() => _MyHomePageState();
}

class _MyHomePageState extends State<MyHomePage> {
  String receivedData;
  double x = 0, y = 0;

  @override
  void initState() {
    widget.socket.listen((List<int> event) {
      print(utf8.decode(event));
      setState(() {
        final receivedData = utf8.decode(event).split(",");

        x = double.tryParse(receivedData[0]) ?? 0;
        y = double.tryParse(receivedData[1]) ?? 0;
      });
    });
    super.initState();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('x: $x, y:$y'),
      ),
      body: Container(
        color: Colors.brown,
        height: MediaQuery.of(context).size.height / 2,
        width: MediaQuery.of(context).size.width,
        child: Circle(x: x, y: y),
      ),
    );
  }
}

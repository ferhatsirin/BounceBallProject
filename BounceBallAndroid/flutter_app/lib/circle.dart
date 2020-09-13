import 'package:flutter/material.dart';
import 'package:meta/meta.dart';

class Circle extends StatelessWidget {

  final double x;
  final double y;

  Circle({this.x, this.y});

  @override
  Widget build(BuildContext context) {
    return new CustomPaint(
      painter: new CirclePainter(x:x, y:y),
    );
  }

}

class CirclePainter extends CustomPainter {

  final double x;
  final double y;
  final _paint = Paint()
    ..color = Colors.red
    ..style = PaintingStyle.fill;

  CirclePainter({this.x, this.y});


  @override
  void paint(Canvas canvas, Size size) {
    canvas.drawOval(
      Rect.fromLTWH(x / 2, y / 2, size.width / 10, size.height / 10),
      _paint,
    );
  }

  @override
  bool shouldRepaint(CirclePainter oldDelegate) => true;

}
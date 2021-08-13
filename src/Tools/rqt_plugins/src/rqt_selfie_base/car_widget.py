#!/usr/bin/env python3

from python_qt_binding.QtWidgets import QGraphicsScene, QGraphicsRectItem, QGraphicsEllipseItem
from python_qt_binding.QtGui import QPen, QBrush
from python_qt_binding.QtCore import Qt, QRectF, QPoint


class CarWidget(QGraphicsScene):
    def __init__(self, x=0, y=0, width=300, height=300, *args):
        super(QGraphicsScene, self).__init__(x, y, width, height, *args)
        self.setBackgroundBrush(QBrush(Qt.gray))

        self._car_width = 150
        self._car_height = 250

        self.car_body = self.addRect(-self._car_width / 2,
                                     -self._car_height / 2, self._car_width,
                                     self._car_height, QPen(Qt.black),
                                     QBrush(Qt.red))
        self.car_body.setX(self.width() / 2)
        self.car_body.setY(self.height() / 2)

        self._wheel_pen = QPen(Qt.black)
        self._wheel_brush = QBrush(Qt.black)

        self.wheel_tr = self._generate_wheel(1, -1)
        self.wheel_tl = self._generate_wheel(-1, -1)
        self.wheel_br = self._generate_wheel(1, 1)
        self.wheel_bl = self._generate_wheel(-1, 1)

    def _generate_wheel(self, x_multiplier: "-1 or 1",
                        y_multiplier: "-1 or 1"):

        wheel_width = self._car_width / 5
        wheel_height = self._car_height / 5
        # add hinges
        rect_hinge = QRectF(0, 0, self._car_width / 10, self._car_width / 10)
        rect_hinge.moveCenter(QPoint(0, 0))
        hinge = QGraphicsEllipseItem(rect_hinge, parent=self.car_body)
        hinge.setX(x_multiplier * self._car_width * 0.5)
        hinge.setY(y_multiplier * self._car_height * 0.35)
        hinge.setBrush(Qt.gray)

        # create wheel with hinge as a parent
        rect_wheel = QRectF(0, 0, wheel_width, wheel_height)
        rect_wheel.moveCenter(QPoint(x_multiplier * wheel_width / 2, 0))
        wheel = QGraphicsRectItem(rect_wheel, parent=hinge)
        wheel.setBrush(self._wheel_brush)
        wheel.setPen(self._wheel_pen)
        return wheel

    def rotate_wheels(self, front_angle=0, back_angle=0):
        self.wheel_tr.setRotation(front_angle)
        self.wheel_tl.setRotation(front_angle)
        self.wheel_br.setRotation(back_angle)
        self.wheel_bl.setRotation(back_angle)

# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'form.ui'
##
## Created by: Qt User Interface Compiler version 6.5.3
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QCheckBox, QGridLayout, QLabel,
    QSizePolicy, QVBoxLayout, QWidget)
import rc_simulator

class Ui_Widget(object):
    def setupUi(self, Widget):
        if not Widget.objectName():
            Widget.setObjectName(u"Widget")
        Widget.resize(800, 534)
        self.verticalLayout_2 = QVBoxLayout(Widget)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.verticalLayout = QVBoxLayout()
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.label = QLabel(Widget)
        self.label.setObjectName(u"label")
        self.label.setPixmap(QPixmap(u":/Downloads/map.png"))
        self.label.setScaledContents(False)
        self.label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.verticalLayout.addWidget(self.label)

        self.gridLayout = QGridLayout()
        self.gridLayout.setObjectName(u"gridLayout")
        self.checkBox_2 = QCheckBox(Widget)
        self.checkBox_2.setObjectName(u"checkBox_2")

        self.gridLayout.addWidget(self.checkBox_2, 0, 1, 1, 1)

        self.checkBox = QCheckBox(Widget)
        self.checkBox.setObjectName(u"checkBox")

        self.gridLayout.addWidget(self.checkBox, 0, 0, 1, 1)

        self.checkBox_3 = QCheckBox(Widget)
        self.checkBox_3.setObjectName(u"checkBox_3")

        self.gridLayout.addWidget(self.checkBox_3, 0, 2, 1, 1)

        self.checkBox_4 = QCheckBox(Widget)
        self.checkBox_4.setObjectName(u"checkBox_4")

        self.gridLayout.addWidget(self.checkBox_4, 1, 0, 1, 1)

        self.checkBox_5 = QCheckBox(Widget)
        self.checkBox_5.setObjectName(u"checkBox_5")

        self.gridLayout.addWidget(self.checkBox_5, 1, 1, 1, 1)

        self.checkBox_6 = QCheckBox(Widget)
        self.checkBox_6.setObjectName(u"checkBox_6")

        self.gridLayout.addWidget(self.checkBox_6, 1, 2, 1, 1)


        self.verticalLayout.addLayout(self.gridLayout)


        self.verticalLayout_2.addLayout(self.verticalLayout)


        self.retranslateUi(Widget)

        QMetaObject.connectSlotsByName(Widget)
    # setupUi

    def retranslateUi(self, Widget):
        Widget.setWindowTitle(QCoreApplication.translate("Widget", u"Widget", None))
        self.label.setText("")
        self.checkBox_2.setText(QCoreApplication.translate("Widget", u"Room1 Light", None))
        self.checkBox.setText(QCoreApplication.translate("Widget", u"Entrance Light", None))
        self.checkBox_3.setText(QCoreApplication.translate("Widget", u"Room2 Light", None))
        self.checkBox_4.setText(QCoreApplication.translate("Widget", u"Kitchen Light", None))
        self.checkBox_5.setText(QCoreApplication.translate("Widget", u"LivingRoom AirConditioner", None))
        self.checkBox_6.setText(QCoreApplication.translate("Widget", u"TV", None))
    # retranslateUi


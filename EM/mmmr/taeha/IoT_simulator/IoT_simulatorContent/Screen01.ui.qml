

/*
This is a UI file (.ui.qml) that is intended to be edited in Qt Design Studio only.
It is supposed to be strictly declarative and only uses a subset of QML. If you edit
this file manually, you might introduce QML code that is not supported by Qt Design Studio.
Check out https://doc.qt.io/qtcreator/creator-quick-ui-forms.html for details on .ui.qml files.
*/
import QtQuick
import QtQuick.Controls
import IoT_simulator

Rectangle {
    id: rectangle
    width: Constants.width
    height: Constants.height
    color: Constants.backgroundColor


    /*
    Button {
        id: button
        text: qsTr("Press me")
        anchors.verticalCenter: parent.verticalCenter
        anchors.verticalCenterOffset: 248
        anchors.horizontalCenterOffset: 459
        checkable: true
        anchors.horizontalCenter: parent.horizontalCenter

        Connections {
            target: button
            onClicked: {
                map_curtain.QObject()
            }
        }
    }
*/
    Switch {
        id: switch_livingroomLight
        x: 893
        y: 80
        text: qsTr("LivingRoom Light")
    }

    Switch {
        id: switch_TV
        x: 893
        y: 160
        text: qsTr("TV")
    }

    Switch {
        id: switch_airConditioner
        x: 893
        y: 240
        text: qsTr("AirConditioner")
    }

    Switch {
        id: switch_airPurifier
        x: 893
        y: 320
        text: qsTr("AirPurifier")
    }

    Switch {
        id: switch_curtain
        x: 893
        y: 400
        text: qsTr("LivingRoom Curtain")
    }

    Image {
        id: map_home
        x: 67
        y: 100
        width: 732
        height: 520
        source: "images/map.png"
        fillMode: Image.PreserveAspectFit
    }

    Image {
        id: image1
        x: 67
        y: 100
        width: 732
        height: 520
        source: "images/livingroomLight.png"
        fillMode: Image.PreserveAspectFit
        visible: switch_livingroomLight.checked
    }

    Image {
        id: area_airConditioner
        x: 67
        y: 100
        width: 732
        height: 520
        source: "images/airconditioner.png"
        fillMode: Image.PreserveAspectFit
        visible: switch_airConditioner.checked
    }

    Image {
        id: area_airPurifier
        x: 67
        y: 100
        width: 732
        height: 520
        source: "images/airpurifier.png"
        fillMode: Image.PreserveAspectFit
        visible: switch_airPurifier.checked
    }

    Image {
        id: area_TV
        x: 67
        y: 100
        width: 732
        height: 520
        source: "images/TV.png"
        fillMode: Image.PreserveAspectFit
        visible: switch_TV.checked
    }

    Image {
        id: area_curtain
        x: 67
        y: 100
        width: 732
        height: 520
        source: "images/curtain.png"
        fillMode: Image.PreserveAspectFit
        visible: switch_curtain.checked
    }

    states: [
        State {
            name: "clicked"
            when: button.checked
        }
    ]
}

import QtQuick
import QtQuick.Window
import QtQuick.Layouts
import QtQuick.Controls

Window {
    width: 800
    height: 480
    visible: true
    title: qsTr("IoT Control Simulator")


    GridLayout {
        anchors.fill: parent
        anchors.margins: 20
        rowSpacing: 20
        columnSpacing: 10
        columns: 3

        Item {
            Layout.leftMargin: 30
            width: 460
            height: 320

            Image {
                id: map_home
                fillMode: Image.PreserveAspectFit
                width: parent.width
                source: "qrc:/images/map.png"
            }

            Image {
                id: area_livingroomLight
                fillMode: Image.PreserveAspectFit
                width: parent.width
                source: "qrc:/images/livingroomLight.png"
                visible: false
            }

            Image {
                id: area_airConditioner
                fillMode: Image.PreserveAspectFit
                width: parent.width
                source: "qrc:/images/airconditioner.png"
                visible: false
            }

            Image {
                id: area_airPurifier
                fillMode: Image.PreserveAspectFit
                width: parent.width
                source: "qrc:/images/airpurifier.png"
                visible: false
            }

            Image {
                id: area_TV
                fillMode: Image.PreserveAspectFit
                width: parent.width
                source: "qrc:/images/TV.png"
                visible: false
            }

            Image {
                id: area_curtain
                fillMode: Image.PreserveAspectFit
                width: parent.width
                source: "qrc:/images/curtain.png"
                visible: false
            }
        }


        ColumnLayout {
            anchors {
                top: parent.top
                topMargin: 30
            }

            spacing: 10

            Switch {
                text: qsTr("LivingRoom Light")
                checked: false
                onClicked: area_livingroomLight.visible = checked
            }

            Switch {
                text: qsTr("TV")
                checked: false
                onClicked: area_TV.visible = checked
            }

            Switch {
                text: qsTr("Air Conditioner")
                checked: false
                onClicked: area_airConditioner.visible = checked
            }

            Switch {
                text: qsTr("Air Purifier")
                checked: false
                onClicked: area_airPurifier.visible = checked
            }

            Switch {
                text: qsTr("LivingRoom Curtain")
                checked: false
                onClicked: area_curtain.visible = checked
            }
        }
    }
}

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
            id: map_wrapper
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
                topMargin: 20
            }
            id: sw_wrapper
            spacing: 7

            Switch {
                id: sw_livingroomLight
                text: qsTr("LivingRoom Light")
                checked: false
                onClicked: area_livingroomLight.visible = checked
            }

            Switch {
                id: sw_TV
                text: qsTr("TV")
                checked: false
                onClicked: area_TV.visible = checked
            }

            Switch {
                id: sw_airConditioner
                text: qsTr("Air Conditioner")
                checked: false
                onClicked: area_airConditioner.visible = checked
            }

            Switch {
                id: sw_airPurifier
                text: qsTr("Air Purifier")
                checked: false
                onClicked: area_airPurifier.visible = checked
            }

            Switch {
                id: sw_curtain
                text: qsTr("LivingRoom Curtain")
                checked: false
                onClicked: area_curtain.visible = checked
            }

            TextArea {
                id: jsonInput
                placeholderText: qsTr("Enter json format")
                Layout.topMargin: 20
                wrapMode: TextArea.WordWrap

                background: Rectangle {
                    implicitWidth: 200
                    implicitHeight: 70
                    color: control.enabled ? "transparent" : "#353637"
                    border.color: jsonInput.enabled ? "#21be2b" : "transparent"
                }
            }

            Button {
                anchors.right: parent.right
                text: "Ok"
                implicitWidth: 100
                onClicked: {
                    var jsonText = jsonInput.text;
                    var result = jsonProcessor.processJson(jsonText);
                    if (result.error) {
                        jsonOutput.text = "Error: " + result.error;
                    } else {
                        jsonOutput.text = result.device + ", " + result.state

                        if(result.state !== "ON" && result.state !== "OFF") {
                            jsonOutput.text = "Error: Invalid device state"
                        }
                        else {
                            if (result.device === "livingroomLight") {
                                area_livingroomLight.visible = result.state === "ON" ? true : false;
                                sw_livingroomLight.checked = result.state === "ON" ? true : false;
                            }
                            else if (result.device === "airConditioner") {
                                area_airConditioner.visible = result.state === "ON" ? true : false;
                                sw_airConditioner.checked = result.state === "ON" ? true : false;
                            }
                            else if (result.device === "airPurifier") {
                                area_airPurifier.visible = result.state === "ON" ? true : false;
                                sw_airPurifier.checked = result.state === "ON" ? true : false;
                            }
                            else if (result.device === "TV") {
                                area_TV.visible = result.state === "ON" ? true : false;
                                sw_TV.checked = result.state === "ON" ? true : false;
                            }
                            else if (result.device === "curtain") {
                                area_curtain.visible = result.state === "ON" ? true : false;
                                sw_curtain.checked = result.state === "ON" ? true : false;
                            }
                            else {
                                jsonOutput.text = "Error: Can't find " + result.device
                            }
                        }
                    }
                }
            }

            TextArea {
                id: jsonOutput
                width: parent.width * 0.8
                height: 200
                wrapMode: TextArea.Wrap
                placeholderText: "Parsed JSON will appear here..."
            }
        }
    }
}

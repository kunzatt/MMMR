import QtQuick
import QtQuick.Window
import QtQuick.Layouts
import QtQuick.Controls
import QtQuick.Controls.Basic
import QtWebSockets

Window {
    width: 800
    height: 480
    visible: true
    title: qsTr("IoT Control Simulator")

    WebSocket {
        id: webSocket
        url: "ws://localhost:12345"
        active: false

        onStatusChanged: {
            if (webSocket.status === WebSocket.Open) {
                console.log("WebSocket connected!");

                // 연결 완료 후 메시지 전송
                var jsonMessage = JSON.stringify({ type: "register", client_type: "receiver" });
                webSocket.sendTextMessage(jsonMessage);
                console.log("Sent: " + jsonMessage);
            }
        }

        onTextMessageReceived: (message) => {
            console.log("Received: " + message)

            var json = JSON.parse(message)
            if (json.type === "ack") {
                console.log("receiver " + json.message)
            }
            else if(json.type === "control") {
                controlDevices(json.device, json.state)
            }
        }
    }

    Component.onCompleted: {
        console.log("Attempting to connect WebSocket...");
        webSocket.active = true;  // 프로그램 실행 시 자동 연결
    }

    function controlDevices(devName, devState) {
        if(devState !== "ON" && devState !== "OFF") {
            jsonOutput.text = "Error: Invalid device state"
        }
        else {
            if (devName === "livingroomLight") {
                area_livingroomLight.visible = devState === "ON" ? true : false;
                sw_livingroomLight.checked = devState === "ON" ? true : false;
            }
            else if (devName === "airConditioner") {
                area_airConditioner.visible = devState === "ON" ? true : false;
                sw_airConditioner.checked = devState === "ON" ? true : false;
            }
            else if (devName === "airPurifier") {
                area_airPurifier.visible = devState === "ON" ? true : false;
                sw_airPurifier.checked = devState === "ON" ? true : false;
            }
            else if (devName === "TV") {
                area_TV.visible = devState === "ON" ? true : false;
                sw_TV.checked = devState === "ON" ? true : false;
            }
            else if (devName === "curtain") {
                area_curtain.visible = devState === "ON" ? true : false;
                sw_curtain.checked = devState === "ON" ? true : false;
            }
            else {
                jsonOutput.text = "Error: Can't find " + devName
            }
        }
    }

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
            Layout.alignment: Qt.AlignTop
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
                    border.color: jsonInput.enabled ? "#21be2b" : "transparent"
                }
            }

            Button {
                Layout.alignment: Qt.AlignRight
                text: "Ok"
                implicitWidth: 100
                onClicked: {
                    var jsonText = jsonInput.text;
                    var result = jsonProcessor.processJson(jsonText);
                    if (result.error) {
                        jsonOutput.text = "Error: " + result.error;
                    } else {
                        jsonOutput.text = result.device + ", " + result.state

                        controlDevices(result.device, result.state)
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

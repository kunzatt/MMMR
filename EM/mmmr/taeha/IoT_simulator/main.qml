import QtQuick
import QtQuick.Window
import QtQuick.Layouts
import QtQuick.Controls
import QtQuick.Controls.Basic
import QtQuick.Shapes
import QtQuick.Effects
import QtWebSockets
import RoomLight
import TV
import AirConditioner
import AirPurifier
import Curtain

ApplicationWindow {
    id: main_window
    width: 1280
    height: 720
    visible: true
    title: qsTr("IoT Control Simulator")

    Text {
        id: main_text
        color: "#ddd"
        font.family: "Titillium Web"
    }

    flags: Qt.FramelessWindowHint | Qt.Window // 기본 제목 표시줄 숨기기

    Rectangle {
        id: titleBar
        width: parent.width
        height: 30
        color: "#3f3f3f"  // 제목 표시줄 배경색

        gradient: Gradient {
            GradientStop { position: 0.9; color: "#3f3f3f" }   // 위쪽 색상
            GradientStop { position: 1.0; color: "#4a4a4a" }    // 아래쪽 색상
        }

        MouseArea {
            id: dragArea
            anchors.fill: parent
            property point clickPos

            onPressed: (mouse) => {
                clickPos = Qt.point(mouse.x, mouse.y)
            }

            onPositionChanged: (mouse) => {
                if (mouse.buttons & Qt.LeftButton) {
                    main_window.setX(mouse.x + main_window.x - clickPos.x)
                    main_window.setY(mouse.y + main_window.y - clickPos.y)
                }
            }

            onDoubleClicked: {
                if (main_window.visibility === Window.Maximized) {
                    main_window.visibility = Window.Windowed  // 창 크기 복원
                    maxwindow_text.text = "🗖";
                } else {
                    main_window.visibility = Window.Maximized  // 최대화
                    maxwindow_text.text = "⿻";
                }
            }
        }

        RowLayout {
            anchors.fill: parent
            spacing: 10

            Label {
                text: "IoT Control Simulator"
                color: "white"
                Layout.alignment: Qt.AlignCenter
                Layout.leftMargin: 10
                font.pixelSize: 12
                font.family: "Titillium Web"
            }
        }

        Row {
            anchors.right: parent.right
            anchors.verticalCenter: parent.verticalCenter
            rightPadding: 5
            spacing: 5

            // 최소화 버튼
            MouseArea {
                width: 20
                height: 20
                onClicked: main_window.showMinimized()  // 최소화 기능

                Rectangle {
                    width: parent.width
                    height: parent.height
                    color: "#1d1d1d"
                    visible: parent.containsMouse ? true : false
                }

                Text {
                    text: "_"
                    anchors.centerIn: parent
                    font.pixelSize: 20
                    color: "white"
                }
            }

            // 최대화 버튼
            MouseArea {
                width: 20
                height: 20

                onClicked: Qt.callLater(() => {
                    if (main_window.visibility === Window.Maximized) {
                        main_window.showNormal();
                        maxwindow_text.text = "🗖";
                    } else {
                        main_window.showMaximized();
                        maxwindow_text.text = "⿻";
                    }
                })

                Rectangle {
                    width: parent.width
                    height: parent.height
                    color: "#1d1d1d"
                    visible: parent.containsMouse ? true : false
                }

                Text {
                    id: maxwindow_text
                    text: "🗖"
                    anchors.centerIn: parent
                    font.pixelSize: 15
                    color: "white"
                }
            }

            // 닫기 버튼
            MouseArea {
                width: 20
                height: 20
                onClicked: Qt.callLater(() => Qt.quit())  // 종료 기능

                Rectangle {
                    width: parent.width
                    height: parent.height
                    color: "#1d1d1d"
                    visible: parent.containsMouse ? true : false

                }

                Text {
                    text: "×"
                    font.bold: true
                    anchors.centerIn: parent
                    font.pixelSize: 23
                    color: "red"
                }
            }
        }
    }

    Rectangle {
        anchors.top: titleBar.bottom
        width: parent.width
        height: parent.height - titleBar.height
        color: "#1f2023"
    }

    /* WebSocket connection */
    WebSocket {
        id: webSocket
        url: "ws://70.12.246.31:12345"
        active: false

        onStatusChanged: {
            if (webSocket.status === WebSocket.Open) {
                console.log("WebSocket connected!");

                // 연결 완료 후 메시지 전송
                var jsonMessage = JSON.stringify({ type: "register", client_type: "iot" });
                webSocket.sendTextMessage(jsonMessage);
                console.log("Sent: " + jsonMessage);
            }
        }

        onTextMessageReceived: (message) => {
            var json = JSON.parse(message)
            if (json.type === "ack") {
                console.log("Received: " + json.message);
                status_message.text = "WebSocket successfully connected";
            }
            else if(json.type === "control") {
                status_message.text = "Received: " + message;
                controlDevices(json.device, json.data)
            }
        }
    }

    Component.onCompleted: {
        status_message.text = "Attempting to connect WebSocket...";
        webSocket.active = true;  // 프로그램 실행 시 자동 연결
    }

    function controlDevices(devName, devData) {
        console.log(devData)
        if(devData["turned"] !== "ON" && devData["turned"] !== "OFF") {
            jsonOutput.text = "Error: Invalid device state"
        }
        else {
            if (devName === "livingroomLight") {
                livingLight.imgId.visible = devData["turned"] === "ON" ? true : false;
                sw_livingLight.checked = devData["turned"] === "ON" ? true : false;
            }
            else if (devName === "airConditioner") {
                livingAirCon.imgId.visible = devData["turned"] === "ON" ? true : false;
                sw_airConditioner.checked = devData["turned"] === "ON" ? true : false;
            }
            else if (devName === "airPurifier") {
                livingAirPurifier.imgId.visible = devData["turned"] === "ON" ? true : false;
                sw_airPurifier.checked = devData["turned"] === "ON" ? true : false;
            }
            else if (devName === "TV") {
                livingTV.imgId.visible = devData["turned"] === "ON" ? true : false;
                sw_TV.checked = devData["turned"] === "ON" ? true : false;
            }
            else if (devName === "curtain") {
                livingCurtain.imgId.visible = devData["turned"] === "ON" ? true : false;
                sw_curtain.checked = devData["turned"] === "ON" ? true : false;
            }
            else {
                jsonOutput.text = "Error: Can't find " + devName
            }
        }
    }

    /* GUI Layout */

    Label {
        id: status_message
        width: 200
        height: 20
        x: 720
        y: 500
        text: "status"
        color: main_text.color
    }

    ColumnLayout {
        id: main_layout
        Layout.alignment: Qt.AlignHCenter
        width: parent.width
        height: parent.height
        spacing: 10

        RowLayout {
            id: sub_layout
            width: parent.width
            Layout.alignment: Qt.AlignHCenter
            spacing: 30

            Item {
                id: map_wrapper
                width: 600
                height: 400

                TV {
                    id: livingTV
                    width: parent.width
                    height: parent.height
                    imgId.source: "qrc:/images/TV.png"
                }

                RoomLight {
                    id: livingLight
                    width: parent.width
                    height: parent.height
                    opacity: 0
                    imgId.source: "qrc:/images/livingroomLight.png"
                    imgId.visible: false
                    visible: false

                    Shape {
                        width: 300
                        height: 300
                        x: 230
                        y: -30

                        ShapePath {
                            strokeWidth: 2
                            strokeColor: "transparent"
                            fillGradient: RadialGradient {
                                centerX: 50; centerY: 150
                                centerRadius: 150
                                focalX: centerX; focalY: centerY
                                GradientStop { position: 0; color: "#efe4b0" }
                                GradientStop { position: 0.8; color: "transparent" }
                            }

                            startX: 50; startY: 0
                            PathArc {
                                x: 50
                                y: 300
                                radiusX: 150
                                radiusY: 150
                            }
                            PathArc {
                                x: 50
                                y: 0
                                radiusX: 150
                                radiusY: 150
                            }
                        }
                    }
                }

                OpacityAnimator {
                    id: fadeIn
                    target: livingLight
                    from: 0
                    to: 1
                    duration: 500
                }

                OpacityAnimator {
                    id: fadeOut
                    target: livingLight
                    from: 1
                    to: 0
                    duration: 500
                }

                RoomLight {
                    id: kitchenLight
                    width: parent.width
                    height: parent.height
                    imgId.source: "qrc:/images/livingroomLight.png"
                    imgId.visible: false
                    visible: false

                    Shape {
                        width: 300
                        height: 300
                        x: 250
                        y: 250

                        ShapePath {
                            strokeWidth: 2
                            strokeColor: "transparent"
                            fillGradient: RadialGradient {
                                centerX: 50; centerY: 100
                                centerRadius: 100
                                focalX: centerX; focalY: centerY
                                GradientStop { position: 0; color: "#efe4b0" }
                                GradientStop { position: 0.8; color: "transparent" }
                            }

                            startX: 50; startY: 0
                            PathArc {
                                x: 50
                                y: 200
                                radiusX: 100
                                radiusY: 100
                            }
                            PathArc {
                                x: 50
                                y: 0
                                radiusX: 100
                                radiusY: 100
                            }
                        }
                    }
                }

                RoomLight {
                    id: entranceLight
                    width: parent.width
                    height: parent.height
                    imgId.source: "qrc:/images/livingroomLight.png"
                    imgId.visible: false
                    visible: false

                    Shape {
                        width: 300
                        height: 300
                        x: 380
                        y: 230

                        ShapePath {
                            strokeWidth: 2
                            strokeColor: "transparent"
                            fillGradient: RadialGradient {
                                centerX: 50; centerY: 100
                                centerRadius: 70
                                focalX: centerX; focalY: centerY
                                GradientStop { position: 0; color: "#efe4b0" }
                                GradientStop { position: 0.8; color: "transparent" }
                            }

                            startX: 50; startY: 0
                            PathArc {
                                x: 50
                                y: 200
                                radiusX: 100
                                radiusY: 100
                            }
                            PathArc {
                                x: 50
                                y: 0
                                radiusX: 100
                                radiusY: 100
                            }
                        }
                    }
                }

                AirConditioner {
                    id: livingAirCon
                    width: parent.width
                    height: parent.height
                    imgId.source: "qrc:/images/airconditioner.png"
                }

                AirPurifier {
                    id: livingAirPurifier
                    width: parent.width
                    height: parent.height
                    imgId.source: "qrc:/images/airpurifier.png"
                }

                Curtain {
                    id: livingCurtain
                    width: parent.width
                    height: parent.height
                    imgId.source: "qrc:/images/curtain.png"
                }

                Image {
                    id: map_home
                    fillMode: Image.PreserveAspectFit
                    width: parent.width
                    source: "qrc:/images/map_white.png"
                }
            }

            ColumnLayout {
                spacing: 30
                width: 500
                Layout.alignment: Qt.AlignHCenter

                Text {
                    text: Qt.formatDate(new Date(), "☀️ yyyy.MM.dd (ddd)");
                    font.pointSize: 30
                    font.bold: true
                    font.family: main_text.font.family
                    color: "#ddd"
                }

                ColumnLayout {
                    Text {
                        text: "Smarthome Info"
                        color: main_text.color
                        font.pointSize: 18
                        font.family: main_text.font.family
                    }

                    Text {
                        text: "Livingroom Light: " + (livingLight.visible ? "ON" : "OFF")
                        color: main_text.color
                        font.pointSize: 13
                        font.family: main_text.font.family
                    }

                    Text {
                        text: "Kitchen Light: " + (kitchenLight.visible ? "ON" : "OFF")
                        color: main_text.color
                        font.pointSize: 13
                        font.family: main_text.font.family
                    }

                    Text {
                        text: "Entrance Light: " + (entranceLight.visible ? "ON" : "OFF")
                        color: main_text.color
                        font.pointSize: 13
                        font.family: main_text.font.family
                    }

                }
            }
        }


        Shape {

            ShapePath {
                strokeWidth: 1
                strokeColor: "lightgray"

                startX: 50
                startY: 0
                PathLine {
                    x: main_window.width - 50 // 화면의 오른쪽에서 50만큼 떨어진 위치
                    y: 0
                }
            }
        }

        RowLayout {
            Layout.alignment: Qt.AlignHCenter
            width: parent.width
            id: sw_wrapper
            spacing: 7

            GridLayout {
                columns: 4

                Switch {
                    id: sw_livingLight
                    text: qsTr("LivingRoom Light")
                    checked: false
                    onClicked: {
                        if(checked) {
                            livingLight.visible = checked
                            fadeIn.start()
                        }
                        else {
                            fadeOut.start()
                            livingLight.visible = checked
                        }
                    }

                    contentItem: Text {
                        text: sw_livingLight.text
                        color: "#ddd"
                        font.family: main_text.font.family
                        verticalAlignment: Text.AlignVCenter
                        leftPadding: sw_livingLight.indicator.width + sw_livingLight.spacing
                    }
                }

                Switch {
                    id: sw_kitchenLight
                    text: qsTr("Kitchen Light")
                    checked: false
                    onClicked: kitchenLight.visible = checked

                    contentItem: Text {
                        text: sw_kitchenLight.text
                        color: "#ddd"
                        font.family: main_text.font.family
                        verticalAlignment: Text.AlignVCenter
                        leftPadding: sw_kitchenLight.indicator.width + sw_kitchenLight.spacing
                    }
                }

                Switch {
                    id: sw_entranceLight
                    text: qsTr("Entrance Light")
                    checked: false
                    onClicked: entranceLight.visible = checked

                    contentItem: Text {
                        text: sw_entranceLight.text
                        color: "#ddd"
                        font.family: main_text.font.family
                        verticalAlignment: Text.AlignVCenter
                        leftPadding: sw_entranceLight.indicator.width + sw_entranceLight.spacing
                    }
                }

                Switch {
                    id: sw_TV
                    text: qsTr("TV")
                    checked: false
                    onClicked: livingTV.imgId.visible = checked

                    contentItem: Text {
                        text: sw_TV.text
                        color: "#ddd"
                        font.family: main_text.font.family
                        verticalAlignment: Text.AlignVCenter
                        leftPadding: sw_TV.indicator.width + sw_TV.spacing
                    }
                }

                Switch {
                    id: sw_airConditioner
                    text: qsTr("Air Conditioner")
                    checked: false
                    onClicked: livingAirCon.imgId.visible = checked

                    contentItem: Text {
                        text: sw_airConditioner.text
                        color: "#ddd"
                        font.family: main_text.font.family
                        verticalAlignment: Text.AlignVCenter
                        leftPadding: sw_airConditioner.indicator.width + sw_airConditioner.spacing
                    }
                }

                Switch {
                    id: sw_airPurifier
                    text: qsTr("Air Purifier")
                    checked: false
                    onClicked: livingAirPurifier.imgId.visible = checked

                    contentItem: Text {
                        text: sw_airPurifier.text
                        color: "#ddd"
                        font.family: main_text.font.family
                        verticalAlignment: Text.AlignVCenter
                        leftPadding: sw_airPurifier.indicator.width + sw_airPurifier.spacing
                    }
                }

                Switch {
                    id: sw_curtain
                    text: qsTr("LivingRoom Curtain")
                    checked: false
                    onClicked: livingCurtain.imgId.visible = checked

                    contentItem: Text {
                        text: sw_curtain.text
                        color: "#ddd"
                        font.family: main_text.font.family
                        verticalAlignment: Text.AlignVCenter
                        leftPadding: sw_curtain.indicator.width + sw_curtain.spacing
                    }
                }
            }

            ColumnLayout {
                TextArea {
                    id: jsonInput
                    placeholderText: qsTr("Enter json format")
                    Layout.topMargin: 20
                    wrapMode: TextArea.WordWrap
                    width: 200

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
                            jsonOutput.text = result.device + ", " + result.data

                            controlDevices(result.device, result.data)
                        }
                    }
                }
            }

            Label {
                id: jsonOutput
                width: parent.width * 0.8
                height: 300
                wrapMode: TextArea.Wrap
                text: "Parsed JSON will appear here..."
                color: "#ddd"
            }
        }
    }


}

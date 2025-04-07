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
    width: 1366
    height: 768
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
        z: 2

        gradient: Gradient {
            GradientStop { position: 0.8; color: "#3f3f3f" }   // 위쪽 색상
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

    // 🔹 창 크기 조절 핫존 설정 🔹
    // 상단
    Rectangle {
        width: parent.width - 20
        height: 10
        anchors.top: parent.top
        anchors.horizontalCenter: parent.horizontalCenter
        opacity: 0
        z: 3

        MouseArea {
            anchors.fill: parent
            cursorShape: Qt.SizeVerCursor
            onPressed: main_window.startSystemResize(Qt.TopEdge)
        }
    }

    // 하단
    Rectangle {
        width: parent.width - 20
        height: 5
        anchors.bottom: parent.bottom
        anchors.horizontalCenter: parent.horizontalCenter
        opacity: 0
        z: 3

        MouseArea {
            anchors.fill: parent
            cursorShape: Qt.SizeVerCursor
            onPressed: main_window.startSystemResize(Qt.BottomEdge)
        }
    }

    // 좌측
    Rectangle {
        width: 5
        height: parent.height - 20
        anchors.left: parent.left
        anchors.verticalCenter: parent.verticalCenter
        opacity: 0
        z: 3

        MouseArea {
            anchors.fill: parent
            cursorShape: Qt.SizeHorCursor
            onPressed: main_window.startSystemResize(Qt.LeftEdge)
        }
    }

    // 우측
    Rectangle {
        width: 5
        height: parent.height - 20
        anchors.right: parent.right
        anchors.verticalCenter: parent.verticalCenter
        opacity: 0
        z: 3

        MouseArea {
            anchors.fill: parent
            cursorShape: Qt.SizeHorCursor
            onPressed: main_window.startSystemResize(Qt.RightEdge)
        }
    }

    // 🔹 모서리 크기 조절 (커스텀 처리)
    Rectangle {
        width: 5; height: 5; anchors.bottom: parent.bottom; anchors.right: parent.right; opacity: 0; z: 3;
        MouseArea {
            id: bottomRightCorner
            anchors.fill: parent
            cursorShape: Qt.SizeFDiagCursor

            property bool resizing: false
            property real startX
            property real startY

            onPressed: {
                resizing = true
                startX = mouseX
                startY = mouseY
            }

            onReleased: {
                resizing = false
            }

            onPositionChanged: {
                if (resizing) {
                    let dx = mouseX - startX
                    let dy = mouseY - startY
                    main_window.width += dx
                    main_window.height += dy
                }
            }
        }
    }

    Rectangle {
        width: 5; height: 5; anchors.top: parent.top; anchors.right: parent.right; opacity: 0; z: 3;
        MouseArea {
            id: topRightCorner
            anchors.fill: parent
            cursorShape: Qt.SizeBDiagCursor

            property bool resizing: false
            property real startX
            property real startY

            onPressed: {
                resizing = true
                startX = mouseX
                startY = mouseY
            }

            onReleased: {
                resizing = false
            }

            onPositionChanged: {
                if (resizing) {
                    let dx = mouseX - startX
                    let dy = mouseY - startY
                    main_window.width += dx
                    main_window.y += dy
                    main_window.height -= dy
                }
            }
        }
    }

    Rectangle {
        width: 5; height: 5; anchors.bottom: parent.bottom; anchors.left: parent.left; opacity: 0; z: 3;
        MouseArea {
            id: bottomLeftCorner
            anchors.fill: parent
            cursorShape: Qt.SizeBDiagCursor

            property bool resizing: false
            property real startX
            property real startY

            onPressed: {
                resizing = true
                startX = mouseX
                startY = mouseY
            }

            onReleased: {
                resizing = false
            }

            onPositionChanged: {
                if (resizing) {
                    let dx = mouseX - startX
                    let dy = mouseY - startY
                    main_window.x += dx
                    main_window.width -= dx
                    main_window.height += dy
                }
            }
        }
    }

    Rectangle {
        width: 5; height: 5; anchors.top: parent.top; anchors.left: parent.left; opacity: 0; z: 3;
        MouseArea {
            id: topLeftCorner
            anchors.fill: parent
            cursorShape: Qt.SizeFDiagCursor

            property bool resizing: false
            property real startX
            property real startY

            onPressed: {
                resizing = true
                startX = mouseX
                startY = mouseY
            }

            onReleased: {
                resizing = false
            }

            onPositionChanged: {
                if (resizing) {
                    let dx = mouseX - startX
                    let dy = mouseY - startY
                    main_window.x += dx
                    main_window.width -= dx
                    main_window.y += dy
                    main_window.height -= dy
                }
            }
        }
    }

    /* WebSocket connection */
    WebSocket {
        id: webSocket
        //url: "ws://70.12.246.31:12345"
        url: "ws://127.0.0.1:12345"
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
        socket_msg_txt.text = devName + ", " + devData["turned"] + ", " + devData["value"]
        fadeInOutAnim.start();
        console.log(devData)
        let baseX = 0, baseY = 0;

        if (devName === "livingroomLight") {
            sw_livingLight.checked = devData["turned"];
            bright_livingLight.visible = devData["turned"];
            bright_livingLight.text = "💡 " + devData["value"];

            if(!devData["turned"]) livingLight_change.to = 0
            else livingLight_change.to = devData["value"] / 100;
            livingLight_change.start()
        }
        else if(devName === "kitchenLight") {
            sw_kitchenLight.checked = devData["turned"]
            bright_kitchenLight.visible = devData["turned"];
            bright_kitchenLight.text = "💡 " + devData["value"];

            if(!devData["turned"]) kitchenLight_change.to = 0
            else kitchenLight_change.to = devData["value"] / 100;
            kitchenLight_change.start()
        }
        else if(devName === "entranceLight") {
            sw_entranceLight.checked = devData["turned"]
            bright_entranceLight.visible = devData["turned"];
            bright_entranceLight.text = "💡 " + devData["value"];

            if(!devData["turned"]) entranceLight_change.to = 0
            else entranceLight_change.to = devData["value"] / 100;
            entranceLight_change.start()
        }
        else if (devName === "aircon") {
            sw_aircon.checked = devData["turned"]
            if(devData["turned"]) aircon_on.start()
            else aircon_off.start()

            temp_aircon.visible = devData["turned"]
            temp_aircon.text = "❄️ " + devData["value"] + "℃";

            baseX = 370;
            baseY = 5;
        }
        else if (devName === "airPurifier") {
            sw_purifier.checked = devData["turned"]
            if(devData["turned"]) purifier_on.start()
            else purifier_off.start()

            baseX = 220;
            baseY = 200;
        }
        else if (devName === "TV") {
            sw_tv.checked = devData["turned"]
            if(devData["turned"]) tv_on.start()
            else tv_off.start()

            volume_tv.visible = devData["turned"];
            volume_tv.text = "🔉 " + devData["value"];

            baseX = 360;
            baseY = 120;
        }
        else if (devName === "curtain") {
            sw_curtain.checked = devData["turned"]
            if(devData["turned"]) curtain_on.start()
            else curtain_off.start()

            baseX = 280;
            baseY = 5;
        }
        else {
            jsonOutput.text = "Error: Can't find " + devName
        }

        if(baseX != 0 && baseY != 0) {
            highlight_circle.opacity = 1
            highlight_circle.width = 10
            highlight_circle.height = 10
            highlight_circle.x = baseX - highlight_circle.width / 2
            highlight_circle.y = baseY - highlight_circle.width / 2
            highlight_circle.scale = 1.0
            scaleCircle.start()
            fadeoutCircle.start()
        }
    }

    /* GUI Layout */
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
                width: 650
                height: 400

                Image {
                    id: map_home
                    fillMode: Image.PreserveAspectFit
                    width: parent.width
                    source: "qrc:/images/smarthome.png"
                }

                TV {
                    id: livingTV
                    //imgId.width: 20
                    imgId.height: 180
                    imgId.x: 360
                    imgId.y: 29
                }

                Image {
                    id: livingTVLight
                    fillMode: Image.PreserveAspectFit
                    width: livingTV.imgId.width
                    height: livingTV.imgId.height
                    x: livingTV.imgId.x
                    y: livingTV.imgId.y
                    source: "qrc:/images/img_TVLight.png"
                    opacity: 0
                }

                OpacityAnimator {
                    id: tv_on
                    target: livingTVLight
                    from: 0
                    to: 1
                    duration: 200
                }

                OpacityAnimator {
                    id: tv_off
                    target: livingTVLight
                    from: 1
                    to: 0
                    duration: 200
                }

                AirConditioner {
                    id: livingAircon
                    imgId.width: 70
                    imgId.height: 70
                    imgId.x: 340
                    imgId.y: -17
                    imgId.visible: true
                }

                Image {
                    id: livingAirconPower
                    fillMode: Image.PreserveAspectFit
                    width: livingAircon.imgId.width
                    height: livingAircon.imgId.height
                    x: livingAircon.imgId.x
                    y: livingAircon.imgId.y
                    source: "qrc:/images/img_airConditionerPower.png"
                    opacity: 0
                }

                OpacityAnimator {
                    id: aircon_on
                    target: livingAirconPower
                    from: 0
                    to: 1
                    duration: 500
                }

                OpacityAnimator {
                    id: aircon_off
                    target: livingAirconPower
                    from: 1
                    to: 0
                    duration: 500
                }


                AirPurifier {
                    id: airPurifier
                    imgId.width: 50
                    imgId.height: 50
                    imgId.x: 193
                    imgId.y: 185
                    imgId.visible: true
                }

                Image {
                    id: airPurifierPower
                    fillMode: Image.PreserveAspectFit
                    width: airPurifier.imgId.width
                    height: airPurifier.imgId.height
                    x: airPurifier.imgId.x
                    y: airPurifier.imgId.y
                    source: "qrc:/images/img_airPurifierPower.png"
                    opacity: 0
                }

                OpacityAnimator {
                    id: purifier_on
                    target: airPurifierPower
                    from: 0
                    to: 1
                    duration: 300
                }

                OpacityAnimator {
                    id: purifier_off
                    target: airPurifierPower
                    from: 1
                    to: 0
                    duration: 300
                }

                Curtain {
                    id: livingCurtain
                    imgId.width: 100
                    imgId.height: 100
                    imgId.x: 228
                    imgId.y: -43
                    visible: true
                }

                Image {
                    id: curtainLight
                    fillMode: Image.PreserveAspectFit
                    width: livingCurtain.imgId.width
                    height: livingCurtain.imgId.height
                    x: livingCurtain.imgId.x - 10
                    y: livingCurtain.imgId.y + 52
                    source: "qrc:/images/CurtainLight.png"
                    opacity: 0
                }

                OpacityAnimator {
                    id: curtain_on
                    target: curtainLight
                    from: 0
                    to: 1
                    duration: 300
                }

                OpacityAnimator {
                    id: curtain_off
                    target: curtainLight
                    from: 1
                    to: 0
                    duration: 300
                } 

                Shape {
                    id: livingLight
                    width: 300
                    height: 300
                    x: 230
                    y: -30
                    opacity: 0.0

                    ShapePath {
                        strokeWidth: 2
                        strokeColor: "transparent"
                        fillGradient: RadialGradient {
                            centerX: 50; centerY: 150
                            centerRadius: 150
                            focalX: centerX; focalY: centerY
                            GradientStop { position: 0; color: "white" }
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

                NumberAnimation {
                    id: livingLight_change
                    target: livingLight
                    property: "opacity"
                    duration: 500
                }

                Shape {
                    id: kitchenLight
                    width: 300
                    height: 300
                    x: 280
                    y: 270
                    opacity: 0.0

                    ShapePath {
                        strokeWidth: 2
                        strokeColor: "transparent"
                        fillGradient: RadialGradient {
                            centerX: 50; centerY: 100
                            centerRadius: 120
                            focalX: centerX; focalY: centerY
                            GradientStop { position: 0; color: "white" }
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

                NumberAnimation {
                    id: kitchenLight_change
                    target: kitchenLight
                    property: "opacity"
                    duration: 500
                }

                Shape {
                    id: entranceLight
                    width: 300
                    height: 300
                    x: 420
                    y: 250
                    opacity: 0.0

                    ShapePath {
                        strokeWidth: 2
                        strokeColor: "transparent"
                        fillGradient: RadialGradient {
                            centerX: 50; centerY: 100
                            centerRadius: 70
                            focalX: centerX; focalY: centerY
                            GradientStop { position: 0; color: "white" }
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

                NumberAnimation {
                    id: entranceLight_change
                    target: entranceLight
                    property: "opacity"
                    duration: 500
                }

                NumberAnimation {
                    id: scaleCircle
                    to: 25.0
                    duration: 700
                    target: highlight_circle
                    properties: "scale"
                    easing.type: Easing.OutCubic
                }

                NumberAnimation {
                    id: fadeoutCircle
                    to: 0.0
                    duration: 700
                    target: highlight_circle
                    properties: "opacity"
                    easing.type: Easing.OutCubic
                }

                Rectangle {
                    id: highlight_circle
                    width: 10
                    height: 10
                    radius: width / 2
                    color: "transparent"
                    border.color: "red"
                    border.width: 0.5
                    transformOrigin: Item.Center
                    opacity: 0
                }
            }

            ColumnLayout {
                spacing: 30
                Layout.alignment: Qt.AlignCenter
                Layout.maximumWidth: 360
                Layout.maximumHeight: 380
                Layout.leftMargin: 40

                ColumnLayout {
                    Text {
                        text: Qt.formatDate(new Date(), "yyyy.MM.dd (ddd)");
                        font.pointSize: 28
                        font.bold: true
                        font.family: main_text.font.family
                        color: "#ddd"
                        Layout.bottomMargin: 20
                    }

                    Text {
                        text: "Lights"
                        color: main_text.color
                        font.pointSize: 18
                        font.family: main_text.font.family
                    }

                    ColumnLayout {
                        Layout.bottomMargin: 20

                        RowLayout {
                            Text {
                                id: power_livingLight
                                text: sw_livingLight.checked ? "🟢" : "🔴"
                                color: main_text.color
                                font.pointSize: 13
                                font.family: main_text.font.family
                            }

                            Text {
                                text: "Living Room"
                                color: main_text.color
                                font.pointSize: 13
                                font.family: main_text.font.family
                            }

                            Text {
                                id: bright_livingLight
                                text: "💡 50"
                                color: main_text.color
                                font.pointSize: 13
                                font.family: main_text.font.family
                                Layout.leftMargin: 20
                                visible: false
                            }
                        }

                        RowLayout {
                            Text {
                                id: power_kitchenLight
                                text: sw_kitchenLight.checked ? "🟢" : "🔴"
                                color: main_text.color
                                font.pointSize: 13
                                font.family: main_text.font.family
                            }

                            Text {
                                text: "Kitchen"
                                color: main_text.color
                                font.pointSize: 13
                                font.family: main_text.font.family
                            }

                            Text {
                                id: bright_kitchenLight
                                text: "💡 50"
                                color: main_text.color
                                font.pointSize: 13
                                font.family: main_text.font.family
                                Layout.leftMargin: 56
                                visible: false
                            }
                        }

                        RowLayout {
                            Text {
                                id: power_entranceLight
                                text: sw_entranceLight.checked ? "🟢" : "🔴"
                                color: main_text.color
                                font.pointSize: 13
                                font.family: main_text.font.family
                            }

                            Text {
                                text: "Entrance"
                                color: main_text.color
                                font.pointSize: 13
                                font.family: main_text.font.family
                            }

                            Text {
                                id: bright_entranceLight
                                text: "💡 50"
                                color: main_text.color
                                font.pointSize: 13
                                font.family: main_text.font.family
                                Layout.leftMargin: 46
                                visible: false
                            }
                        }
                    }

                    Text {
                        text: "Appliances"
                        color: main_text.color
                        font.pointSize: 18
                        font.family: main_text.font.family
                    }

                    ColumnLayout {
                        RowLayout {
                            Text {
                                id: power_aircon
                                text: sw_aircon.checked ? "🟢" : "🔴"
                                color: main_text.color
                                font.pointSize: 13
                                font.family: main_text.font.family
                            }

                            Text {
                                text: "Air Conditioner"
                                color: main_text.color
                                font.pointSize: 13
                                font.family: main_text.font.family
                            }

                            Text {
                                id: temp_aircon
                                text: "❄️ 24℃"
                                color: main_text.color
                                font.pointSize: 13
                                font.family: main_text.font.family
                                Layout.leftMargin: 20
                                visible: false
                            }
                        }

                        RowLayout {
                            Text {
                                id: power_tv
                                text: sw_tv.checked ? "🟢" : "🔴"
                                color: main_text.color
                                font.pointSize: 13
                                font.family: main_text.font.family
                            }

                            Text {
                                text: "TV"
                                color: main_text.color
                                font.pointSize: 13
                                font.family: main_text.font.family
                            }

                            Text {
                                id: volume_tv
                                text: "🔉 50"
                                color: main_text.color
                                font.pointSize: 13
                                font.family: main_text.font.family
                                Layout.leftMargin: 112
                                visible: false
                            }
                        }

                        RowLayout {
                            Text {
                                id: power_purifier
                                text: sw_purifier.checked ? "🟢" : "🔴"
                                color: main_text.color
                                font.pointSize: 13
                                font.family: main_text.font.family
                            }

                            Text {
                                text: "Air Purifier"
                                color: main_text.color
                                font.pointSize: 13
                                font.family: main_text.font.family
                            }
                        }

                        RowLayout {
                            Text {
                                id: power_curtain
                                text: sw_curtain.checked ? "🟢" : "🔴"
                                color: main_text.color
                                font.pointSize: 13
                                font.family: main_text.font.family
                            }

                            Text {
                                text: "Curtain"
                                color: main_text.color
                                font.pointSize: 13
                                font.family: main_text.font.family
                            }
                        }
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
            Layout.alignment: Qt.AlignHCenter | Qt.AlignTop
            Layout.maximumHeight: 100
            width: parent.width
            id: sw_wrapper
            spacing: 40

            GridLayout {
                columns: 4

                Switch {
                    id: sw_livingLight
                    text: qsTr("LivingRoom Light")
                    checked: false
                    onClicked: {
                        let msg = JSON.stringify({ type: "send", device: "livingroomLight", data: {"turned": sw_livingLight.checked, "value": 100} })
                        webSocket.sendTextMessage(msg)
                    }

                    indicator: Rectangle {
                        implicitWidth: 40
                        implicitHeight: 20
                        x: sw_livingLight.leftPadding
                        y: parent.height / 2 - height / 2
                        color: sw_livingLight.checked ? "#666" : "#ffffff"
                        border.color: sw_livingLight.checked ? "#666" : "#cccccc"

                        Rectangle {
                            x: sw_livingLight.checked ? parent.width - width : 0
                            width: 15
                            height: 20
                            color: sw_livingLight.down ? "#cccccc" : "#ffffff"
                            border.color: sw_livingLight.checked ? (sw_livingLight.down ? "#666" : "#888") : "#999999"

                            Behavior on x {
                                NumberAnimation {
                                    duration: 200
                                    easing.type: Easing.OutQuad
                                }
                            }
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
                    onClicked: {
                        let msg = JSON.stringify({ type: "send", device: "kitchenLight", data: {"turned": sw_kitchenLight.checked, "value": 100} })
                        webSocket.sendTextMessage(msg)
                    }

                    indicator: Rectangle {
                        implicitWidth: 40
                        implicitHeight: 20
                        x: sw_kitchenLight.leftPadding
                        y: parent.height / 2 - height / 2
                        color: sw_kitchenLight.checked ? "#666" : "#ffffff"
                        border.color: sw_kitchenLight.checked ? "#666" : "#cccccc"

                        Rectangle {
                            x: sw_kitchenLight.checked ? parent.width - width : 0
                            width: 15
                            height: 20
                            color: sw_kitchenLight.down ? "#cccccc" : "#ffffff"
                            border.color: sw_kitchenLight.checked ? (sw_kitchenLight.down ? "#666" : "#888") : "#999999"

                            Behavior on x {
                                NumberAnimation {
                                    duration: 200
                                    easing.type: Easing.OutQuad
                                }
                            }
                        }
                    }

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
                    onClicked: {
                        let msg = JSON.stringify({ type: "send", device: "entranceLight", data: {"turned": sw_entranceLight.checked, "value": 100} })
                        webSocket.sendTextMessage(msg)
                    }

                    indicator: Rectangle {
                        implicitWidth: 40
                        implicitHeight: 20
                        x: sw_entranceLight.leftPadding
                        y: parent.height / 2 - height / 2
                        color: sw_entranceLight.checked ? "#666" : "#ffffff"
                        border.color: sw_entranceLight.checked ? "#666" : "#cccccc"

                        Rectangle {
                            x: sw_entranceLight.checked ? parent.width - width : 0
                            width: 15
                            height: 20
                            color: sw_entranceLight.down ? "#cccccc" : "#ffffff"
                            border.color: sw_entranceLight.checked ? (sw_entranceLight.down ? "#666" : "#888") : "#999999"

                            Behavior on x {
                                NumberAnimation {
                                    duration: 200
                                    easing.type: Easing.OutQuad
                                }
                            }
                        }
                    }

                    contentItem: Text {
                        text: sw_entranceLight.text
                        color: "#ddd"
                        font.family: main_text.font.family
                        verticalAlignment: Text.AlignVCenter
                        leftPadding: sw_entranceLight.indicator.width + sw_entranceLight.spacing
                    }
                }

                Switch {
                    id: sw_tv
                    text: qsTr("TV")
                    checked: false
                    onClicked: {
                        let msg = JSON.stringify({ type: "send", device: "TV", data: {"turned": sw_tv.checked, "value": 0} })
                        webSocket.sendTextMessage(msg)
                    }

                    indicator: Rectangle {
                        implicitWidth: 40
                        implicitHeight: 20
                        x: sw_tv.leftPadding
                        y: parent.height / 2 - height / 2
                        color: sw_tv.checked ? "#666" : "#ffffff"
                        border.color: sw_tv.checked ? "#666" : "#cccccc"

                        Rectangle {
                            x: sw_tv.checked ? parent.width - width : 0
                            width: 15
                            height: 20
                            color: sw_tv.down ? "#cccccc" : "#ffffff"
                            border.color: sw_tv.checked ? (sw_tv.down ? "#666" : "#888") : "#999999"

                            Behavior on x {
                                NumberAnimation {
                                    duration: 200
                                    easing.type: Easing.OutQuad
                                }
                            }
                        }
                    }

                    contentItem: Text {
                        text: sw_tv.text
                        color: "#ddd"
                        font.family: main_text.font.family
                        verticalAlignment: Text.AlignVCenter
                        leftPadding: sw_tv.indicator.width + sw_tv.spacing
                    }
                }

                Switch {
                    id: sw_aircon
                    text: qsTr("Air Conditioner")
                    checked: false
                    onClicked: {
                        let msg = JSON.stringify({ type: "send", device: "aircon", data: {"turned": sw_aircon.checked, "value": 0} })
                        webSocket.sendTextMessage(msg)
                    }

                    indicator: Rectangle {
                        implicitWidth: 40
                        implicitHeight: 20
                        x: sw_aircon.leftPadding
                        y: parent.height / 2 - height / 2
                        color: sw_aircon.checked ? "#666" : "#ffffff"
                        border.color: sw_aircon.checked ? "#666" : "#cccccc"

                        Rectangle {
                            x: sw_aircon.checked ? parent.width - width : 0
                            width: 15
                            height: 20
                            color: sw_aircon.down ? "#cccccc" : "#ffffff"
                            border.color: sw_aircon.checked ? (sw_aircon.down ? "#666" : "#888") : "#999999"

                            Behavior on x {
                                NumberAnimation {
                                    duration: 200
                                    easing.type: Easing.OutQuad
                                }
                            }
                        }
                    }

                    contentItem: Text {
                        text: sw_aircon.text
                        color: "#ddd"
                        font.family: main_text.font.family
                        verticalAlignment: Text.AlignVCenter
                        leftPadding: sw_aircon.indicator.width + sw_aircon.spacing
                    }
                }

                Switch {
                    id: sw_purifier
                    text: qsTr("Air Purifier")
                    checked: false
                    onClicked: {
                        let msg = JSON.stringify({ type: "send", device: "airPurifier", data: {"turned": sw_purifier.checked, "value": 0} })
                        webSocket.sendTextMessage(msg)
                    }

                    indicator: Rectangle {
                        implicitWidth: 40
                        implicitHeight: 20
                        x: sw_purifier.leftPadding
                        y: parent.height / 2 - height / 2
                        color: sw_purifier.checked ? "#666" : "#ffffff"
                        border.color: sw_purifier.checked ? "#666" : "#cccccc"

                        Rectangle {
                            x: sw_purifier.checked ? parent.width - width : 0
                            width: 15
                            height: 20
                            color: sw_purifier.down ? "#cccccc" : "#ffffff"
                            border.color: sw_purifier.checked ? (sw_purifier.down ? "#666" : "#888") : "#999999"

                            Behavior on x {
                                NumberAnimation {
                                    duration: 200
                                    easing.type: Easing.OutQuad
                                }
                            }
                        }
                    }

                    contentItem: Text {
                        text: sw_purifier.text
                        color: "#ddd"
                        font.family: main_text.font.family
                        verticalAlignment: Text.AlignVCenter
                        leftPadding: sw_purifier.indicator.width + sw_purifier.spacing
                    }
                }

                Switch {
                    id: sw_curtain
                    text: qsTr("LivingRoom Curtain")
                    checked: false
                    onClicked: {
                        let msg = JSON.stringify({ type: "send", device: "curtain", data: {"turned": sw_curtain.checked, "value": 0} })
                        webSocket.sendTextMessage(msg)
                    }

                    indicator: Rectangle {
                        implicitWidth: 40
                        implicitHeight: 20
                        x: sw_curtain.leftPadding
                        y: parent.height / 2 - height / 2
                        color: sw_curtain.checked ? "#666" : "#ffffff"
                        border.color: sw_curtain.checked ? "#666" : "#cccccc"

                        Rectangle {
                            x: sw_curtain.checked ? parent.width - width : 0
                            width: 15
                            height: 20
                            color: sw_curtain.down ? "#cccccc" : "#ffffff"
                            border.color: sw_curtain.checked ? (sw_curtain.down ? "#666" : "#888") : "#999999"

                            Behavior on x {
                                NumberAnimation {
                                    duration: 200
                                    easing.type: Easing.OutQuad
                                }
                            }
                        }
                    }

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
                Layout.maximumWidth: 200
                Layout.maximumHeight: 100
                spacing: 5

                TextArea {
                    id: jsonInput
                    placeholderText: qsTr("Enter json format")
                    wrapMode: TextArea.WordWrap
                    implicitWidth: 200
                    implicitHeight: 70

                    background: Rectangle {
                        width: 200
                        height: 70
                        border.color: jsonInput.enabled ? "#21be2b" : "transparent"
                    }
                }

                Button {
                    text: "Ok"
                    Layout.alignment: Qt.AlignRight
                    Layout.preferredWidth: 80
                    Layout.preferredHeight: 30
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
                Layout.preferredWidth: 200
                Layout.preferredHeight: 80
                wrapMode: TextArea.Wrap
                text: "Parsed JSON will appear here..."
                color: "#ddd"
            }
        }
    }

    Rectangle {
        color: "#555"
        width: status_message.implicitWidth + 10
        height: status_message.implicitHeight + 10
        x: 0
        y: main_window.height - 30

        Text {
            id: status_message
            text: "status"
            color: main_text.color
            font.family: main_text.font.family
            font.pointSize: 12
            anchors.centerIn: parent
        }
    }

    SequentialAnimation {
        id: fadeInOutAnim
        running: false
        onStopped: {
            // 완전히 사라졌으면 visible도 false로
            if (socket_msg.opacity === 0.0) {
                socket_msg.visible = false;
                socket_msg.x = 860
            }
        }

        ScriptAction { script: socket_msg.visible = true }

        ParallelAnimation {
            running: false

            NumberAnimation {
                target: socket_msg
                property: "opacity"
                to: 1.0
                duration: 300
            }

            NumberAnimation {
                target: socket_msg
                property: "x"
                to: 830
                duration: 300
            }
        }

        PauseAnimation { duration: 700 }

        ParallelAnimation {
            running: false

            NumberAnimation {
                target: socket_msg
                property: "opacity"
                to: 0.0
                duration: 300
            }

            NumberAnimation {
                target: socket_msg
                property: "x"
                to: 860
                duration: 300
            }
        }
    }

    Rectangle {
        id: socket_msg
        width: 350
        height: 50
        x: 860
        y: 500
        opacity: 0.0

        Text {
            id: socket_msg_txt
            text: "test"
            color: main_text.color
            font.family: main_text.font.family
            font.pointSize: 16
            anchors.left: parent.left
            anchors.verticalCenter: parent.verticalCenter
            anchors.leftMargin: 20
        }

        gradient: Gradient {
            orientation: Gradient.Horizontal

            GradientStop { position: 0.8; color: "#444" }
            GradientStop { position: 1.0; color: "transparent" }
        }
    }
}

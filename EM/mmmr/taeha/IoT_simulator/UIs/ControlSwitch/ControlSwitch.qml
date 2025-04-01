import QtQuick 2.15
import QtQuick.Controls

Item {
    property alias swId: custom_sw
    Switch {
        id: custom_sw
        checked: false

        contentItem: Text {
            text: custom_sw.text
            color: "#ddd"
            font.family: main_text.font.family
            verticalAlignment: Text.AlignVCenter
            leftPadding: custom_sw.indicator.width + custom_sw.spacing
        }

        indicator: Rectangle {
            implicitWidth: 40
            implicitHeight: 20
            x: sw_livingLight.leftPadding
            y: parent.height / 2 - height / 2
            color: sw_livingLight.checked ? "gray" : "#ffffff"
            border.color: sw_livingLight.checked ? "gray" : "#cccccc"

            Rectangle {
                x: sw_livingLight.checked ? parent.width - width : 0
                width: 15
                height: 20
                color: sw_livingLight.down ? "#cccccc" : "#ffffff"
                border.color: sw_livingLight.checked ? "gray" : "#cccccc"

                Behavior on x {
                    NumberAnimation {
                        duration: 200  // 애니메이션 지속 시간 (ms)
                        easing.type: Easing.OutQuad  // 부드러운 가속도 적용
                    }
                }
            }
        }
    }
}

import QtQuick 2.15

Item {
    id: light
    property alias imgId: area_light

    Image {
        id: area_light
        fillMode: Image.PreserveAspectFit
        width: parent.width
        height: parent.height
        source: "qrc:/images/light.png"
        visible: false
    }
}

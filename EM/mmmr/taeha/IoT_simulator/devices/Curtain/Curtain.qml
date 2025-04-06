import QtQuick 2.15

Item {
    id: curtain

    property alias imgId: area_curtain

    Image {
        id: area_curtain
        fillMode: Image.PreserveAspectFit
        width: parent.width
        height: parent.height
        source: "qrc:/images/curtain.png"
    }
}

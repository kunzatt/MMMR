import QtQuick 2.15

Item {
    id: tv
    // property bool turned: false
    // property int intensity: 10

    property alias imgId: area_airPurifier

    Image {
        id: area_airPurifier
        fillMode: Image.PreserveAspectFit
        width: parent.width
        height: parent.height
        source: "qrc:/images/light.png"
        visible: false
    }
}

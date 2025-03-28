import QtQuick 2.15

Item {
    id: tv
    // property bool turned: false
    // property int volume: 100
    // property int channelNum: 1

    property alias imgId: area_tv

    Image {
        id: area_tv
        fillMode: Image.PreserveAspectFit
        width: parent.width
        height: parent.height
        source: "qrc:/images/light.png"
        visible: false
    }
}

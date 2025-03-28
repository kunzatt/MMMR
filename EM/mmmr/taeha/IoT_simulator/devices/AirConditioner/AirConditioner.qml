import QtQuick 2.15

Item {
    id: airConditioner
    // property bool turned: false
    // property int temperature: 25
    // property int channelNum: 1

    property alias imgId: area_airConditioner

    Image {
        id: area_airConditioner
        fillMode: Image.PreserveAspectFit
        width: parent.width
        height: parent.height
        source: "qrc:/images/light.png"
        visible: false
    }
}

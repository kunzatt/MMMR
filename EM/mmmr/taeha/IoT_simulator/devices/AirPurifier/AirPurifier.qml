import QtQuick 2.15

Item {
    property alias imgId: airPurifier

    Image {
        id: airPurifier
        fillMode: Image.PreserveAspectFit
        source: "qrc:/images/img_airPurifier.png"
    }
}

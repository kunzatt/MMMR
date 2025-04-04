import QtQuick 2.15

Item {
    id: airConditioner
    property alias imgId: area_airConditioner

    Image {
        id: area_airConditioner
        fillMode: Image.PreserveAspectFit
        source: "qrc:/images/img_airConditioner.png"
        visible: true
    }
}

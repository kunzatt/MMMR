import QtQuick 2.15

Item {
    id: tv
    property alias imgId: area_tv

    Image {
        id: area_tv
        fillMode: Image.PreserveAspectFit
        source: "qrc:/images/img_TV.png"
    }
}

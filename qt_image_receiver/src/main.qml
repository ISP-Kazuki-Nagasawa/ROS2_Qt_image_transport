import QtQuick 2.9
import QtQuick.Window 2.2

Window {
    visible: true
    width: 640
    height: 480
    title: qsTr("Qt image receiver")

    Rectangle {
        width: parent.width
        height: parent.height
        Image {
            id: image
            source: "image://VideoCapture/0"
        }
        Timer {
            property int cnt: 0
            interval: 32
            running: true
            repeat: true
            onTriggered: {
                if (image.status === Image.Ready) {
                    image.source = "image://VideoCapture/" + cnt;
                    cnt++;
                }
            }
        }
    }
}

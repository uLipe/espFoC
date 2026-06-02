import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import espFoC.theme

Item {
    id: root
    Rectangle {
        anchors.fill: parent
        color: Theme.windowBg
    }
    Column {
        anchors.centerIn: parent
        spacing: 16
        Image {
            anchors.horizontalCenter: parent.horizontalCenter
            source: scope.logoUrl
            sourceSize.width: 160
            sourceSize.height: 160
            width: 160
            height: 160
            fillMode: Image.PreserveAspectFit
            smooth: true
        }
        BusyIndicator {
            anchors.horizontalCenter: parent.horizontalCenter
            implicitWidth: 28
            implicitHeight: 28
            Material.theme: Material.Dark
            Material.accent: Material.Cyan
            running: true
        }
        Label {
            anchors.horizontalCenter: parent.horizontalCenter
            text: scope.splashMessage
            font.pixelSize: 11
            color: Theme.textMuted
            opacity: 0.9
        }
    }
}

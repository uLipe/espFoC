import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts
import espFoC.theme

Item {
    id: root

    Rectangle {
        anchors.fill: parent
        color: Theme.windowBg
    }

    Image {
        anchors.centerIn: parent
        width: Math.min(parent.width, parent.height) * 0.72
        height: width
        source: scope.logoUrl
        fillMode: Image.PreserveAspectFit
        opacity: Theme.watermarkOpacity
        smooth: true
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 16
        spacing: Theme.spacing

        Label {
            Layout.fillWidth: true
            text: scope.statusText
            font.pixelSize: 12
            color: Theme.textMuted
            horizontalAlignment: Text.AlignRight
        }

        ScrollView {
            Layout.fillWidth: true
            Layout.fillHeight: true
            clip: true
            ScrollBar.vertical: ScrollBar { policy: ScrollBar.AsNeeded }

            Flow {
                width: parent.width
                spacing: Theme.spacing

                Repeater {
                    model: scope.plotModel
                    delegate: ChannelPlotCard {
                        width: Math.max(400, (parent.width - Theme.spacing) / 2 - Theme.spacing)
                        height: 300
                    }
                }
            }
        }
    }

    RoundButton {
        id: fab
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        anchors.margins: 24
        width: 56
        height: 56
        radius: Theme.radiusFab
        Material.background: Theme.primary
        Material.foreground: "#121212"
        font.pixelSize: 28
        text: "+"
        onClicked: scope.openAddChannelDialog()
    }
}

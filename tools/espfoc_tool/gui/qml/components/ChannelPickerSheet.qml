import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts
import espFoC.theme

Dialog {
    id: root
    title: "Add channel"
    modal: true
    anchors.centerIn: Overlay.overlay
    width: Math.min(480, Overlay.overlay.width - 48)
    standardButtons: Dialog.NoButton
    Material.theme: Material.Dark
    Material.accent: Material.Cyan

    property int selectedChannel: -1
    property string selectedColor: scope.defaultPlotColor

    background: Rectangle {
        color: Theme.surfaceBg
        radius: Theme.radiusCard
        border.color: Theme.outline
    }

    contentItem: ColumnLayout {
        spacing: Theme.spacing

        Label {
            text: "Line color"
            font.pixelSize: 13
            color: Theme.textMuted
        }
        Flow {
            Layout.fillWidth: true
            spacing: 10
            Repeater {
                model: scope.colorPalette
                delegate: Rectangle {
                    width: 36
                    height: 36
                    radius: 18
                    color: modelData
                    border.width: root.selectedColor === modelData ? 3 : 1
                    border.color: root.selectedColor === modelData ? Theme.textPrimary : Theme.outline
                    TapHandler {
                        onTapped: root.selectedColor = modelData
                    }
                }
            }
        }

        Label {
            text: "Channel"
            font.pixelSize: 13
            color: Theme.textMuted
        }
        ListView {
            Layout.fillWidth: true
            Layout.preferredHeight: 280
            clip: true
            spacing: 4
            model: scope.channelCatalog
            delegate: ItemDelegate {
                width: ListView.view.width
                enabled: !modelData.inUse
                text: "Ch " + modelData.index + "  " + modelData.name + "  [" + modelData.unit + "]"
                highlighted: root.selectedChannel === modelData.index
                onClicked: root.selectedChannel = modelData.index
                Material.theme: Material.Dark
            }
        }

        RowLayout {
            Layout.fillWidth: true
            Item { Layout.fillWidth: true }
            Button {
                text: "Cancel"
                flat: true
                onClicked: root.reject()
            }
            Button {
                text: "Add plot"
                highlighted: true
                enabled: root.selectedChannel >= 0
                Material.background: Theme.primary
                onClicked: {
                    scope.addPlot(root.selectedChannel, root.selectedColor)
                    root.accept()
                }
            }
        }
    }
}

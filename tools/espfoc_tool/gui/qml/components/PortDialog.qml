import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts
import espFoC.theme

Dialog {
    id: root
    title: "Scope stream port"
    modal: true
    anchors.centerIn: parent
    standardButtons: Dialog.NoButton
    Material.theme: Material.Dark
    Material.accent: Material.Cyan

    property alias portField: portBox.editText
    property alias baudField: baudBox.editText

    background: Rectangle {
        color: Theme.surfaceBg
        radius: Theme.radiusCard
        border.color: Theme.outline
    }

    contentItem: ColumnLayout {
        spacing: Theme.spacing
        width: 400
        Label {
            text: "USB Serial/JTAG scope port — not the UART used for idf monitor."
            wrapMode: Text.WordWrap
            color: Theme.textMuted
            font.pixelSize: 13
            Layout.fillWidth: true
        }
        ComboBox {
            id: portBox
            Layout.fillWidth: true
            model: scope.portList
            editable: true
            Material.theme: Material.Dark
        }
        ComboBox {
            id: baudBox
            Layout.fillWidth: true
            model: ["921600", "460800", "115200"]
            currentIndex: 0
            editable: true
            Material.theme: Material.Dark
        }
        RowLayout {
            Layout.fillWidth: true
            spacing: Theme.spacing
            Button {
                text: "Refresh"
                flat: true
                onClicked: scope.refreshPorts()
            }
            Item { Layout.fillWidth: true }
            Button {
                text: "Cancel"
                flat: true
                onClicked: root.reject()
            }
            Button {
                text: "Connect"
                highlighted: true
                Material.background: Theme.primary
                onClicked: {
                    scope.connectPort(portBox.editText, parseInt(baudBox.editText) || 921600)
                    root.accept()
                }
            }
        }
    }
}

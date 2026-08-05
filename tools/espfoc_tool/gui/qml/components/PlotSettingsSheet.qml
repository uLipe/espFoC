import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts
import espFoC.theme

Dialog {
    id: root
    title: "Plot settings"
    modal: true
    anchors.centerIn: Overlay.overlay
    width: Math.min(420, Overlay.overlay.width - 48)
    standardButtons: Dialog.NoButton
    Material.theme: Material.Dark
    Material.accent: Material.Cyan

    property int draftSampleRate: scope.plotSampleRateHz
    property int draftMaxPoints: scope.plotMaxPoints

    function syncFromBackend() {
        draftSampleRate = scope.plotSampleRateHz
        draftMaxPoints = scope.plotMaxPoints
        rateSpin.value = draftSampleRate
        pointsSpin.value = draftMaxPoints
    }

    onAboutToShow: syncFromBackend()

    background: Rectangle {
        color: Theme.surfaceBg
        radius: Theme.radiusCard
        border.color: Theme.outline
    }

    contentItem: ColumnLayout {
        spacing: Theme.spacing

        Label {
            Layout.fillWidth: true
            wrapMode: Text.WordWrap
            text: "Applies to all plot cards. Time window = points ÷ sample rate."
            font.pixelSize: 12
            color: Theme.textMuted
        }

        RowLayout {
            Layout.fillWidth: true
            Label {
                Layout.preferredWidth: 120
                text: "Sample rate"
                color: Theme.textPrimary
            }
            SpinBox {
                id: rateSpin
                Layout.fillWidth: true
                editable: true
                inputMethodHints: Qt.ImhDigitsOnly
                from: 1
                to: 1000
                stepSize: 1
                value: root.draftSampleRate
                onValueModified: root.draftSampleRate = value
                Material.theme: Material.Dark
            }
            Label {
                text: "Hz"
                color: Theme.textMuted
            }
        }

        RowLayout {
            Layout.fillWidth: true
            Label {
                Layout.preferredWidth: 120
                text: "Max points"
                color: Theme.textPrimary
            }
            SpinBox {
                id: pointsSpin
                Layout.fillWidth: true
                editable: true
                inputMethodHints: Qt.ImhDigitsOnly
                from: 10
                to: scope.plotPointsAbsoluteMax
                stepSize: 10
                value: root.draftMaxPoints
                onValueModified: root.draftMaxPoints = value
                Material.theme: Material.Dark
            }
        }

        Label {
            Layout.fillWidth: true
            text: "Time window: " + (root.draftMaxPoints / Math.max(1, root.draftSampleRate)).toFixed(2) + " s"
            font.pixelSize: 12
            color: Theme.textMuted
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
                text: "OK"
                highlighted: true
                Material.background: Theme.primary
                onClicked: {
                    scope.applyPlotSettings(rateSpin.value, pointsSpin.value)
                    root.accept()
                }
            }
        }
    }
}

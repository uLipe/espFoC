import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts
import espFoC.theme

Item {
    id: root

    readonly property int minCardWidth: 360
    readonly property int minCardHeight: 200
    readonly property int maxCardHeight: 320

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

        RowLayout {
            Layout.fillWidth: true
            spacing: Theme.spacing

            Button {
                id: autosetBtn
                text: "Autoset"
                highlighted: true
                font.pixelSize: 12
                Material.background: Theme.surfaceVariant
                Material.foreground: Theme.primary
                Material.elevation: 3
                leftPadding: 14
                rightPadding: 14
                onClicked: scope.autoset()
            }

            ToolButton {
                display: AbstractButton.IconOnly
                Material.foreground: Theme.textMuted
                ToolTip.visible: hovered
                ToolTip.text: "Plot settings"
                onClicked: scope.openPlotSettingsDialog()
                contentItem: Text {
                    text: "\u2699"
                    color: Theme.textMuted
                    font.pixelSize: 20
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
            }

            Label {
                Layout.fillWidth: true
                text: scope.statusText
                font.pixelSize: 12
                color: Theme.textMuted
                horizontalAlignment: Text.AlignRight
                elide: Text.ElideLeft
            }
        }

        ScrollView {
            id: plotScroll
            Layout.fillWidth: true
            Layout.fillHeight: true
            clip: true
            ScrollBar.vertical.policy: ScrollBar.AsNeeded
            ScrollBar.horizontal.policy: ScrollBar.AlwaysOff
            contentWidth: plotContent.width
            contentHeight: plotContent.height

            Item {
                id: plotContent
                width: plotScroll.availableWidth

                property int plotCount: plotRepeater.count
                property int gridColumns: Math.max(1,
                    Math.floor((width + Theme.spacing) / (root.minCardWidth + Theme.spacing)))
                property real cardWidth: gridColumns > 0
                    ? (width - (gridColumns - 1) * Theme.spacing) / gridColumns
                    : width
                property int gridRows: plotCount > 0
                    ? Math.ceil(plotCount / gridColumns)
                    : 0
                property real cardHeight: {
                    if (plotCount <= 0)
                        return root.maxCardHeight
                    var rows = gridRows
                    var avail = plotScroll.height
                    if (avail <= 0)
                        return root.maxCardHeight
                    var fitted = (avail - (rows - 1) * Theme.spacing) / rows
                    return Math.max(root.minCardHeight,
                                    Math.min(root.maxCardHeight, fitted))
                }
                height: plotCount > 0
                    ? gridRows * cardHeight + Math.max(0, gridRows - 1) * Theme.spacing
                    : 0

                GridLayout {
                    id: plotGrid
                    width: parent.width
                    columns: plotContent.gridColumns
                    rowSpacing: Theme.spacing
                    columnSpacing: Theme.spacing

                    Repeater {
                        id: plotRepeater
                        model: scope.plotModel
                        delegate: ChannelPlotCard {
                            Layout.preferredWidth: plotContent.cardWidth
                            Layout.preferredHeight: plotContent.cardHeight
                            Layout.maximumWidth: plotContent.cardWidth
                            Layout.maximumHeight: plotContent.cardHeight
                            width: plotContent.cardWidth
                            height: plotContent.cardHeight
                        }
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

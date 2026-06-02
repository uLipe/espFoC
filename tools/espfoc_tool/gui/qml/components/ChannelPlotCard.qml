import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts
import espFoC.theme

Item {
    id: root
    implicitWidth: 480
    implicitHeight: 300

    required property int plotId
    required property int channelIndex
    required property string plotTitle
    required property string unit
    required property color lineColor
    required property var points
    required property string cursorLabel

    readonly property real emptyMinX: 0
    readonly property real emptyMaxX: 5
    readonly property real emptyMinY: -1
    readonly property real emptyMaxY: 1

    function formatAxis(v) {
        var a = Math.abs(v)
        if (a >= 1000)
            return v.toExponential(1)
        if (a >= 100)
            return v.toFixed(0)
        if (a >= 10)
            return v.toFixed(1)
        if (a >= 1)
            return v.toFixed(2)
        return v.toFixed(3)
    }

    function computeDomain(pts) {
        if (!pts || pts.length === 0) {
            return {
                minX: emptyMinX, maxX: emptyMaxX,
                minY: emptyMinY, maxY: emptyMaxY
            }
        }
        var minX = pts[0].x
        var maxX = pts[0].x
        var minY = pts[0].y
        var maxY = pts[0].y
        for (var i = 1; i < pts.length; i++) {
            minX = Math.min(minX, pts[i].x)
            maxX = Math.max(maxX, pts[i].x)
            minY = Math.min(minY, pts[i].y)
            maxY = Math.max(maxY, pts[i].y)
        }
        if (maxX - minX < 1e-9)
            maxX = minX + 1
        if (maxY - minY < 1e-9) {
            minY -= 0.5
            maxY += 0.5
        }
        return { minX: minX, maxX: maxX, minY: minY, maxY: maxY }
    }

    Rectangle {
        id: card
        anchors.fill: parent
        color: Theme.surfaceBg
        radius: Theme.radiusCard
        border.color: Theme.outline
        border.width: 1

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 12
            spacing: 8

            RowLayout {
                Layout.fillWidth: true
                Rectangle {
                    width: 10
                    height: 10
                    radius: 5
                    color: root.lineColor
                }
                Label {
                    text: root.plotTitle
                    font.pixelSize: 15
                    font.weight: Font.Medium
                    color: Theme.textPrimary
                    Layout.fillWidth: true
                    Layout.minimumWidth: 80
                    elide: Text.ElideRight
                }
                Label {
                    text: root.unit
                    font.pixelSize: 12
                    color: Theme.textMuted
                }
                ToolButton {
                    z: 1
                    display: AbstractButton.IconOnly
                    Material.foreground: Theme.textMuted
                    onClicked: scope.removePlotForChannel(root.channelIndex)
                    contentItem: Text {
                        text: "✕"
                        color: Theme.textMuted
                        font.pixelSize: 16
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                }
            }

            Item {
                Layout.fillWidth: true
                Layout.fillHeight: true

                Canvas {
                    id: plotCanvas
                    anchors.fill: parent

                    readonly property int padL: 52
                    readonly property int padR: 10
                    readonly property int padT: 10
                    readonly property int padB: 30
                    readonly property int gridDiv: 4

                    property real cursorX: -1
                    property real cursorY: -1
                    property real dataMinX: root.emptyMinX
                    property real dataMaxX: root.emptyMaxX
                    property real dataMinY: root.emptyMinY
                    property real dataMaxY: root.emptyMaxY

                    function plotWidth() { return width - padL - padR }
                    function plotHeight() { return height - padT - padB }

                    function mapX(x, minX, maxX) {
                        var w = plotWidth()
                        return padL + (x - minX) / (maxX - minX) * w
                    }

                    function mapY(y, minY, maxY) {
                        var h = plotHeight()
                        return padT + (1 - (y - minY) / (maxY - minY)) * h
                    }

                    function drawGrid(ctx, minX, maxX, minY, maxY) {
                        var w = plotWidth()
                        var h = plotHeight()
                        ctx.strokeStyle = "#2a2b2f"
                        ctx.lineWidth = 1
                        for (var gx = 0; gx <= gridDiv; gx++) {
                            var xx = padL + gx / gridDiv * w
                            ctx.beginPath()
                            ctx.moveTo(xx, padT)
                            ctx.lineTo(xx, padT + h)
                            ctx.stroke()
                        }
                        for (var gy = 0; gy <= gridDiv; gy++) {
                            var yy = padT + gy / gridDiv * h
                            ctx.beginPath()
                            ctx.moveTo(padL, yy)
                            ctx.lineTo(padL + w, yy)
                            ctx.stroke()
                        }
                        ctx.strokeStyle = "#3d3e42"
                        ctx.lineWidth = 1.5
                        ctx.beginPath()
                        ctx.moveTo(padL, padT + h)
                        ctx.lineTo(padL + w, padT + h)
                        ctx.stroke()
                        ctx.beginPath()
                        ctx.moveTo(padL, padT)
                        ctx.lineTo(padL, padT + h)
                        ctx.stroke()
                    }

                    function drawAxisLabels(ctx, minX, maxX, minY, maxY) {
                        var w = plotWidth()
                        var h = plotHeight()
                        ctx.fillStyle = "#9aa0a6"
                        ctx.font = "10px monospace"
                        ctx.textAlign = "center"
                        ctx.textBaseline = "top"
                        for (var gx = 0; gx <= gridDiv; gx++) {
                            var tx = minX + gx / gridDiv * (maxX - minX)
                            var xx = padL + gx / gridDiv * w
                            ctx.fillText(root.formatAxis(tx), xx, padT + h + 5)
                        }
                        ctx.textAlign = "right"
                        ctx.textBaseline = "middle"
                        for (var gy = 0; gy <= gridDiv; gy++) {
                            var ty = maxY - gy / gridDiv * (maxY - minY)
                            var yy = padT + gy / gridDiv * h
                            ctx.fillText(root.formatAxis(ty), padL - 6, yy)
                        }
                        ctx.textAlign = "left"
                        ctx.textBaseline = "alphabetic"
                        ctx.font = "9px sans-serif"
                        ctx.fillStyle = "#6b7075"
                        ctx.fillText("t (s)", padL + w - 28, padT + h + 5)
                        if (root.unit.length)
                            ctx.fillText(root.unit, 2, padT + 10)
                    }

                    function drawTrace(ctx, pts, minX, maxX, minY, maxY) {
                        if (!pts || pts.length < 2)
                            return
                        ctx.strokeStyle = root.lineColor
                        ctx.lineWidth = 2
                        ctx.beginPath()
                        ctx.moveTo(mapX(pts[0].x, minX, maxX), mapY(pts[0].y, minY, maxY))
                        for (var k = 1; k < pts.length; k++)
                            ctx.lineTo(mapX(pts[k].x, minX, maxX), mapY(pts[k].y, minY, maxY))
                        ctx.stroke()
                    }

                    function drawCrosshair(ctx) {
                        if (cursorX < 0)
                            return
                        var w = plotWidth()
                        var h = plotHeight()
                        ctx.strokeStyle = "#9aa0a6"
                        ctx.lineWidth = 1
                        ctx.setLineDash([4, 4])
                        ctx.beginPath()
                        ctx.moveTo(cursorX, padT)
                        ctx.lineTo(cursorX, padT + h)
                        ctx.stroke()
                        ctx.beginPath()
                        ctx.moveTo(padL, cursorY)
                        ctx.lineTo(padL + w, cursorY)
                        ctx.stroke()
                        ctx.setLineDash([])
                    }

                    onPaint: {
                        var ctx = getContext("2d")
                        ctx.reset()
                        ctx.fillStyle = "#18191c"
                        ctx.fillRect(0, 0, width, height)

                        var dom = root.computeDomain(root.points)
                        dataMinX = dom.minX
                        dataMaxX = dom.maxX
                        dataMinY = dom.minY
                        dataMaxY = dom.maxY

                        if (plotWidth() <= 0 || plotHeight() <= 0)
                            return

                        drawGrid(ctx, dom.minX, dom.maxX, dom.minY, dom.maxY)
                        drawAxisLabels(ctx, dom.minX, dom.maxX, dom.minY, dom.maxY)
                        drawTrace(ctx, root.points, dom.minX, dom.maxX, dom.minY, dom.maxY)
                        drawCrosshair(ctx)
                    }

                    onWidthChanged: requestPaint()
                    onHeightChanged: requestPaint()
                    Component.onCompleted: requestPaint()
                }

                Connections {
                    target: root
                    function onPointsChanged() { plotCanvas.requestPaint() }
                }

                MouseArea {
                    anchors.fill: parent
                    hoverEnabled: true
                    onPositionChanged: function(mouse) {
                        var c = plotCanvas
                        var w = c.plotWidth()
                        var h = c.plotHeight()
                        if (w <= 0 || h <= 0)
                            return
                        var mx = mouse.x
                        var my = mouse.y
                        if (mx < c.padL || mx > c.padL + w || my < c.padT || my > c.padT + h) {
                            c.cursorX = -1
                            c.cursorY = -1
                            scope.clearCursorForPlot(root.plotId)
                            c.requestPaint()
                            return
                        }
                        var fx = (mx - c.padL) / w
                        var fy = 1 - (my - c.padT) / h
                        c.cursorX = mx
                        c.cursorY = my
                        var xVal = c.dataMinX + fx * (c.dataMaxX - c.dataMinX)
                        var yVal = c.dataMinY + fy * (c.dataMaxY - c.dataMinY)
                        scope.setCursorForPlot(root.plotId, xVal, yVal)
                        c.requestPaint()
                    }
                    onExited: {
                        plotCanvas.cursorX = -1
                        plotCanvas.cursorY = -1
                        scope.clearCursorForPlot(root.plotId)
                        plotCanvas.requestPaint()
                    }
                }
            }

            Label {
                Layout.fillWidth: true
                text: root.cursorLabel.length ? root.cursorLabel : "t=—  y=—"
                font.family: "monospace"
                font.pixelSize: 11
                color: Theme.textMuted
                elide: Text.ElideRight
            }
        }
    }
}

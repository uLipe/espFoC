import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import espFoC.theme
import "components"

ApplicationWindow {
    id: window
    visible: true
    width: 1200
    height: 800
    title: "espFoC Tool"
    color: Theme.windowBg
    Material.theme: Material.Dark
    Material.accent: Material.Cyan

    property bool portDialogOpen: false

    StackView {
        id: stack
        anchors.fill: parent
        initialItem: splashComponent
    }

    Component {
        id: splashComponent
        SplashView {}
    }

    Component {
        id: mainComponent
        MainShell {}
    }

    PortDialog {
        id: portDlg
        onRejected: Qt.quit()
    }

    ChannelPickerSheet {
        id: channelDlg
    }

    Connections {
        target: scope
        function onNavigateTo(screenName) {
            if (screenName === "splash")
                stack.replace(splashComponent)
            else if (screenName === "main")
                stack.replace(mainComponent)
        }
        function onRequestPortDialog() {
            portDlg.open()
        }
        function onRequestAddChannelDialog() {
            channelDlg.selectedChannel = -1
            channelDlg.selectedColor = scope.defaultPlotColor
            channelDlg.open()
        }
    }

    Component.onCompleted: scope.startup()
}

pragma Singleton
import QtQuick

QtObject {
    readonly property color windowBg: "#121212"
    readonly property color surfaceBg: "#1e1f22"
    readonly property color surfaceVariant: "#26272b"
    readonly property color outline: "#3a3b3f"
    readonly property color textPrimary: "#e6e6e6"
    readonly property color textMuted: "#9aa0a6"
    readonly property color primary: "#4fc3f7"
    readonly property color error: "#cf6679"

    readonly property int radiusCard: 16
    readonly property int radiusFab: 28
    readonly property int spacing: 12
    readonly property real watermarkOpacity: 0.07
}

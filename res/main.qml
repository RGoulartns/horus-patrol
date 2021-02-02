import QtQuick 2.12
import QtQuick.Controls 2.5
import "view"
import "form"

ApplicationWindow {
    id: window
    width: 640
    height: 480
    visible: true
    title: qsTr("Horus Patrol")

    property string user: ""
    property int appFont: Qt.application.font.pixelSize * 1.1

    Loader {
        id: userAccessScreen
        anchors.centerIn: parent
        width: 300
        height: 300
        sourceComponent: UserAccess {
            id: userAccess
            anchors.fill: parent
            textFont: appFont
            onLoginSuccessful: {
                user = username
                userAccessScreen.active = false
            }
        }
    }
    header: ToolBar {
        contentHeight: toolButton.implicitHeight
        visible: user ? true : false
        font.pixelSize: appFont

        ToolButton {
            id: toolButton
            text: stackView.depth > 1 ? "\u25C0" : "\u2630"
            font.pixelSize: Qt.application.font.pixelSize * 1.6
            onClicked: {
                if (stackView.depth > 1) {
                    stackView.pop()
                } else {
                    drawer.open()
                }
            }
        }

        Label {
            text: stackView.currentItem.title
            anchors.centerIn: parent
        }
    }

    Drawer {
        id: drawer
        width: Math.min(window.width * 0.33, appFont * 15)
        height: window.height
        font.pixelSize: appFont

        Column {
            anchors.fill: parent

            ItemDelegate {
                text: qsTr("Mapping")
                width: parent.width
                onClicked: {
                    stackView.push("qrc:form/Mapping.ui.qml")
                    drawer.close()
                }
            }
            ItemDelegate {
                text: qsTr("Patrolling")
                width: parent.width
                onClicked: {
                    stackView.push("qrc:form/Patrolling.ui.qml")
                    drawer.close()
                }
            }
        }
    }

    StackView {
        id: stackView
        visible: user ? true : false
        initialItem: "qrc:form/Home.ui.qml"
        anchors.fill: parent
    }
}

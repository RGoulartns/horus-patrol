import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Window 2.12
import Horus.Queries.User 1.0

Page {

    property int textFont: 20
    property string appName: "Horus Patrol"

    signal loginSuccessful(string username)

    UserQueries{
        id: queries
        onQueryResult: {
            if(result) {
                loginSuccessful(usernameTf.text)
            }
            else {
                dialog.__title = "ERROR: INVALID CREDENTIALS"
                dialog.__message = "Invalid username and/or password."
                dialog.open()
                usernameTf.text = ""
                passwordTf.text = ""
                usernameTf.forceActiveFocus()
            }
        }

        Component.onCompleted: {
            if(!queries.dbConnected)
            {
                dialog.__title = "ERROR: Database access"
                dialog.__message = "Unable to connect to database."
                dialog.open()
                //TODO: implement Qt.quit()
            }
            else if(!queries.dbExist) {
                dialog.__title = "INFORMATION: New Database created"
                dialog.__message = "Horus database was not found. A new database has been created along with a default root user"
                dialog.open()
            }
        }
    }

    header:  Label {
        text: "Login to " + appName
        font.pixelSize: textFont
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        anchors {
            top: parent.top
            topMargin: textFont
            horizontalCenter: parent.horizontalCenter
        }
    }

    footer: Rectangle {
        id: rectangle
        anchors.horizontalCenter: parent.horizontalCenter
        height: loginBtn.height
        color: "transparent"
        Button {
            id: loginBtn
            text: "Login"
            font.pixelSize: textFont * 0.75
            width: Math.max(loginBtn.implicitWidth, manageBtn.implicitWidth)
            height: textFont * 2.5
            anchors.left: parent.left

            onClicked: queries.login(usernameTf.text, passwordTf.text)
        }

        Button {
            id: manageBtn
            text: "Manage"
            font.pixelSize: textFont * 0.75
            width: Math.max(loginBtn.implicitWidth, manageBtn.implicitWidth)
            height: textFont * 2.5
            anchors.right: parent.right
        }
    }

    Column {
        anchors.centerIn: parent
        spacing: textFont

        TextField {
            id: usernameTf
            placeholderText: qsTr("Username")
            font.pixelSize: textFont

            implicitWidth: textFont * 11
            implicitHeight: textFont * 2.5
            bottomPadding: textFont * 0.5
            leftPadding: textFont
            rightPadding: textFont

            background: Rectangle {
                color: "white"
                border.color: "lightblue"
                border.width: textFont / 10
                radius: textFont
                smooth: true
                clip: true
            }
        }

        TextField {
            id: passwordTf
            placeholderText: qsTr("Password")
            font.pixelSize: textFont

            implicitWidth: textFont * 11
            implicitHeight: textFont * 2.5
            bottomPadding: textFont * 0.5
            leftPadding: textFont
            rightPadding: textFont

            verticalAlignment: Text.AlignVCenter
            echoMode: TextInput.Password

            background: Rectangle {
                color: "white"
                border.color: "lightblue"
                border.width: textFont / 10
                radius: textFont
                smooth: true
                clip: true
            }
        }
    }

    Dialog {
        property string __title: " "
        property string __message: " "

        id: dialog
        anchors.centerIn: Overlay.overlay
        implicitWidth: Math.min(parent.width, Screen.desktopAvailableWidth)
        implicitHeight: parent.height * 0.75

        title: qsTr(__title)
        modal: true
        focus: true
        closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutsideParent
        standardButtons: Dialog.Ok
        font.pixelSize: textFont
        contentItem: Text {
            id: name
            text: qsTr(dialog.__message)
            wrapMode: Text.WordWrap
            font.pixelSize: textFont * 0.9
        }
    }
}

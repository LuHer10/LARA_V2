import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Window {
    visible: true
    width: 640
    height: 480
    title: qsTr("Dual Joystick UDP App")

    property real joystickSize: height * 6 / 10

    property real leftX: 0
    property real leftY: 0
    property real rightX: 0
    property real rightY: 0
    property int gripEnabled: 0
    property int omni: 0
    property int manualOn: 0

    property int currentScreen: 0

    // Commands for the three degrees of freedom
    property int dof1: 0
    property int dof2: 0
    property int dof3: 0
    property int dof4: 0

    property int home: 0

    Button {
            id: screenButton
            anchors.top: parent.top
            anchors.right: parent.right
            anchors.margins: 10
            z: 1

            text: currentScreen === 0
                  ? "Joint Control"
                  : "Joystick Control"

            onClicked: {
                currentScreen = currentScreen === 0 ? 1 : 0
            }
        }

    StackLayout {
        id: screenLayout
        anchors.fill: parent
        currentIndex: currentScreen

        // SCREEN 1: Joystick controls
        Item {
            id: joystickScreen

            // LEFT JOYSTICK
            Rectangle {
                id: leftJoystick
                anchors.left: parent.left
                anchors.verticalCenter: parent.verticalCenter
                width: joystickSize
                height: joystickSize
                color: "#DDDDDD"

                // Background circle
                Rectangle {
                    id: leftBase
                    anchors.centerIn: parent
                    width: joystickSize
                    height: joystickSize
                    radius: joystickSize / 2
                    color: "#AAAAAA"
                }

                // Joystick knob
                Rectangle {
                    id: leftKnob
                    width: joystickSize / 3
                    height: joystickSize / 3
                    radius: joystickSize / 6
                    color: "#4444FF"
                    x: leftJoystick.width / 2 - width / 2
                    y: leftJoystick.height / 2 - height / 2
                }

                MultiPointTouchArea {
                    anchors.fill: parent
                    touchPoints: [ TouchPoint { id: lt } ]
                    onTouchUpdated: {
                        if (lt.pressed) {
                            let dx = lt.x - leftJoystick.width / 2
                            let dy = lt.y - leftJoystick.height / 2
                            let distance = Math.sqrt(dx * dx + dy * dy)
                            let maxRadius = joystickSize / 2

                            if (distance > maxRadius) {
                                let scale = maxRadius / distance
                                dx *= scale
                                dy *= scale
                            }

                            leftKnob.x = leftJoystick.width / 2 - leftKnob.width / 2 + dx
                            leftKnob.y = leftJoystick.height / 2 - leftKnob.height / 2 + dy

                            leftX = dx / maxRadius
                            leftY = dy / maxRadius

                            if(Math.abs(leftX) < 0.1){
                                leftX = 0
                            }
                            if(Math.abs(leftY) < 0.1){
                                leftY = 0
                            }

                        } else {
                            leftKnob.x = leftJoystick.width / 2 - leftKnob.width / 2
                            leftKnob.y = leftJoystick.height / 2 - leftKnob.height / 2
                            leftX = 0
                            leftY = 0
                        }
                    }
                }

                Text {
                    anchors.centerIn: parent
                    text: "direction"
                    color: "white"
                }
            }

            // RIGHT JOYSTICK
            Rectangle {
                id: rightJoystick
                anchors.right: parent.right
                anchors.verticalCenter: parent.verticalCenter
                width: joystickSize
                height: joystickSize
                color: "#BBBBBB"

                // Background circle
                Rectangle {
                    id: rightBase
                    anchors.centerIn: parent
                    width: joystickSize
                    height: joystickSize
                    radius: parent.height / 2
                    color: "#888888"
                }

                // Joystick knob
                Rectangle {
                    id: rightKnob
                    width: joystickSize / 3
                    height: joystickSize / 3
                    radius: joystickSize / 6
                    color: omni ? "#FF4444" : "#FF6644"
                    x: rightJoystick.width / 2 - width / 2
                    y: rightJoystick.height / 2 - height / 2
                }

                MultiPointTouchArea {
                    anchors.fill: parent
                    touchPoints: [ TouchPoint { id: rt } ]
                    onTouchUpdated: {
                        if (rt.pressed) {
                            let dx = rt.x - rightJoystick.width / 2
                            let dy = rt.y - rightJoystick.height / 2
                            let distance = Math.sqrt(dx * dx + dy * dy)
                            let maxRadius = joystickSize / 2

                            if (distance > maxRadius) {
                                let scale = maxRadius / distance
                                dx *= scale
                                dy *= scale
                            }

                            rightKnob.x = rightJoystick.width / 2 - rightKnob.width / 2 + dx
                            rightKnob.y = rightJoystick.height / 2 - rightKnob.height / 2 + dy

                            rightX = dx / maxRadius
                            rightY = dy / maxRadius
                            if(Math.abs(rightX) < 0.1){
                                rightX = 0
                            }
                            if(Math.abs(rightY) < 0.1){
                                rightY = 0
                            }

                        } else {
                            rightKnob.x = rightJoystick.width / 2 - rightKnob.width / 2
                            rightKnob.y = rightJoystick.height / 2 - rightKnob.height / 2
                            rightX = 0
                            rightY = 0
                        }
                    }
                }

                Text {
                    anchors.centerIn: parent
                    text: omni ? "rotation" : "arm"
                    color: "white"
                }

            }

            Button {
                id: gripButton
                width: 100
                height: 50

                anchors.right: parent.right
                anchors.bottom: parent.bottom
                anchors.rightMargin: 200
                anchors.bottomMargin: 10

                checkable: true
                text: checked ? "Release" : "Grip"

                onCheckedChanged: {
                    gripEnabled = checked
                }
            }

            Button {
                id: omniButton
                width: 100
                height: 50

                anchors.left: parent.left
                anchors.bottom: parent.bottom
                anchors.leftMargin: 200
                anchors.bottomMargin: 10

                checkable: true
                text: checked ? "Diff" : "Omni"

                onCheckedChanged: {
                    omni = checked
                }
            }

        }

        // SCREEN 2: Three DOF controls
        Item {
            id: jointScreen


            RowLayout {
                anchors.centerIn: parent
                spacing: 40


                Repeater {
                    model: 4

                    delegate: ColumnLayout {
                        spacing: 10

                        Label {
                            text: "DOF " + (index + 1)
                            font.pixelSize: 20
                            Layout.alignment: Qt.AlignHCenter
                            color: "black"
                        }

                        Button {
                            text: "+"
                            Layout.preferredWidth: 80
                            Layout.preferredHeight: 55

                            onPressed: {
                                if (index === 0) dof1 = 1
                                else if (index === 1) dof2 = 1
                                else if (index === 2) dof3 = 1
                                else if (index === 3) dof4 = 1
                            }

                            onReleased: {
                                if (index === 0) dof1 = 0
                                else if (index === 1) dof2 = 0
                                else if (index === 2) dof3 = 0
                                else if (index === 3) dof4 = 0
                            }

                            onCanceled: {
                                if (index === 0) dof1 = 0
                                else if (index === 1) dof2 = 0
                                else if (index === 2) dof3 = 0
                                else if (index === 3) dof4 = 0
                            }
                        }

                        Button {
                            text: "-"
                            Layout.preferredWidth: 80
                            Layout.preferredHeight: 55

                            onPressed: {
                                if (index === 0) dof1 = -1
                                else if (index === 1) dof2 = -1
                                else if (index === 2) dof3 = -1
                                else if (index === 3) dof4 = -1
                            }

                            onReleased: {
                                if (index === 0) dof1 = 0
                                else if (index === 1) dof2 = 0
                                else if (index === 2) dof3 = 0
                                else if (index === 3) dof4 = 0
                            }

                            onCanceled: {
                                if (index === 0) dof1 = 0
                                else if (index === 1) dof2 = 0
                                else if (index === 2) dof3 = 0
                                else if (index === 3) dof4 = 0
                            }
                        }
                    }
                }

            }

            Button {
                id: homeButton

                text: "Home"
                width: 100
                height: 50

                anchors.horizontalCenter: parent.horizontalCenter
                anchors.bottom: parent.bottom
                anchors.bottomMargin: 10

                onPressed: {
                    home = 1
                    homeTimer.restart()
                }

                onReleased: {
                    home = 0
                    homeTimer.stop()
                }
            }

        }
    }


    Timer {
        id: sendTimer
        interval: 100
        repeat: true
        running: true
        onTriggered: {
            udpHandler.sendJoystickData(leftX, -leftY, rightX, -rightY, dof1, dof2, dof3, dof4,
                                        currentScreen << 3 | home << 2 | omni << 1 | gripEnabled)
        }
    }

    Timer {
        id: homeTimer
        interval: 100
        repeat: false

        onTriggered: {
            home = 0
        }
    }

    Text {
        id: serverData
        anchors.top: parent.top
        anchors.horizontalCenter: parent.horizontalCenter
        font.pixelSize: 24
        text: "Waiting for server..."
        padding: 10
    }




    // Server data connection
    Connections {
        target: udpHandler
        onDataReceived: serverData.text = data
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;

public class DoorServo {
    ServoController doorServo = new ServoController();
    private HardwareMap hardwareMap;
    private Gamepad gamepad;


    public DoorServo(HardwareMap hardwareMap, Gamepad gamepad) {
        this.gamepad = gamepad;
        doorServo.init(hardwareMap, "doorServo");
    }

    public void loop() {
        if (gamepad.dpad_down) {
            doorServo.setServoPosition(0);
        } else if (gamepad.dpad_up) {
            doorServo.setServoPosition(1);
        } else {
            //doorServo.setServoPosition(-1);
        }
    }

    public void SetState(boolean isOpen) {
        if(isOpen) {
            doorServo.setServoPosition(0);
        } else {
            doorServo.setServoPosition(1);
        }
    }
}
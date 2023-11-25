package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;

public class BucketDoor {
    ServoController doorServo = new ServoController();
    private HardwareMap hardwareMap;
    private Gamepad gamepad;


    public BucketDoor(HardwareMap hardwareMap, Gamepad gamepad) {
        this.gamepad = gamepad;
        doorServo.init(hardwareMap);
    }

    public void loop() {
        if (gamepad.dpad_down) {
            doorServo.setServoPosition(0);
        } else if (gamepad.dpad_up) {
            doorServo.setServoPosition(1);
        } else {
            doorServo.setServoPosition(0.5);
        }
    }

    public void SetPosition(boolean open) {
        if(open) {
            doorServo.setServoPosition(1);
        } else {
            doorServo.setServoPosition(0.5);
        }
    }
}
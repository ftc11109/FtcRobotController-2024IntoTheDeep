package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BucketServo {
    ServoController doorServo = new ServoController();
    private HardwareMap hardwareMap;
    private Gamepad gamepad;


    public BucketServo(HardwareMap hardwareMap, Gamepad gamepad) {
        this.gamepad = gamepad;
        doorServo.init(hardwareMap, "bucketServo");
    }

    public void loop() {
        if (gamepad.dpad_left) {
            doorServo.setServoPosition(0.75);
        } else if (gamepad.dpad_right) {
            doorServo.setServoPosition(0.25);
        } else {
            //doorServo.setServoPosition(-1);
        }
    }

    public void SetState(boolean isDeployed) {
        if(isDeployed) {
            doorServo.setServoPosition(0);
        } else {
            doorServo.setServoPosition(1);
        }
    }
}
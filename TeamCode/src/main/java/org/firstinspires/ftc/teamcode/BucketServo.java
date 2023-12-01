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
        if (SwingArm.currentSetPosition == 1) {
            doorServo.setServoPosition(0);
        } else if (SwingArm.currentSetPosition == 2) {
            doorServo.setServoPosition(0.5);
        } else if (SwingArm.currentSetPosition == 3) {
            doorServo.setServoPosition(1);
        }
    }
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class BucketServo {
    ServoController bucketServo = new ServoController();
    private HardwareMap hardwareMap;

    public BucketServo(HardwareMap hardwareMap) {
        bucketServo.init(hardwareMap, "bucketServo");
        bucketServo.setServoPosition(0);
    }

    public void loop() {
        if (SwingArm.armMotor.getCurrentPosition() < 1500) {
            bucketServo.setServoPosition(0.2);
        } else {
            bucketServo.setServoPosition(((double)(SwingArm.armMotor.getCurrentPosition() - 1500) / 1200));
        }
    }
}
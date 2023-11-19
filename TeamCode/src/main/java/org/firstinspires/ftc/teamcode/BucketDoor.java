package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BucketDoor {
    ServoController board = new ServoController();
    private HardwareMap hardwareMap;

    public void init(HardwareMap hardwareMap) {

        board.init(hardwareMap);
    }
/*
    public void loop() {
        if (SwingArm.currentPosition == 0) {
            board.setServoPosition(2);
        } else if (SwingArm.currentPosition == 0) {
            board.setServoPosition(0);
        } else {
            board.setServoPosition(1);
        }
    } */
}
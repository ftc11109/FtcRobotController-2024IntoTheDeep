package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

public class BucketDoor {

    ServoController door = new ServoController();


    public void init() {
        door.init(hardwareMap);
    }


    public void loop() {

        if (gamepad1.a) {
            door.setServoPosition(1.0);
        } else if (gamepad1.b) {
            door.setServoPosition(0.0);
        } else {
            door.setServoPosition(0.5);
        }
    }
}

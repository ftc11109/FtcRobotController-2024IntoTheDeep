package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BucketDoor {

    ServoController door = new ServoController();

    private final Telemetry telemetry;

    private final Gamepad gamepad;


    public void init() {
        door.init(hardwareMap);
    }

    public BucketDoor(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad) {
        this.telemetry = telemetry;
        this.gamepad = gamepad;
        door = hardwareMap.get(ServoController.class, "door");
    }

    public void loop() {

        if (gamepad2.dpad_up) {
            door.setServoPosition(1.0);
        } else if (gamepad2.dpad_down) {
            door.setServoPosition(0.0);
        } else {
            door.setServoPosition(0.5);
        }
    }
}

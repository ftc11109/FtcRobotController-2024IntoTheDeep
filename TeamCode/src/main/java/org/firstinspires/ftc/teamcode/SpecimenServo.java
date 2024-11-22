package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SpecimenServo {

    private final Gamepad gamepad;
    ServoController specimenServo = new ServoController();

    static boolean isOpen = true;

    static final double OPEN_POSITION = 0.5; // servo physical zero position
    static final double CLOSED_POSITION = 0.15; // todo: test different values
    final boolean isAutonomous;
    public SpecimenServo(HardwareMap hardwareMap, Gamepad gamepad, boolean isAutonomous) {
        this.gamepad = gamepad;
        this.isAutonomous = isAutonomous;
        specimenServo.init(hardwareMap,"specimenServo"); // port 3
    }

    public void loop() {
        if (gamepad.x) {
            isOpen = !isOpen;
            specimenServo.setServoPosition(
                    isOpen ? OPEN_POSITION : CLOSED_POSITION
            );
            while(gamepad.x);
        }

    }

}

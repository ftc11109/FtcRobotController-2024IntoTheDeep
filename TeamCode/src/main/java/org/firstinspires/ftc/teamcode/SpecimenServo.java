package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SpecimenServo {

    private final Gamepad gamepad;
    ServoController rampServo  = new ServoController();

    static final double OPEN_POSITION = 0.5; // servo physical zero position
    static final double CLOSED_POSITION = 0.1; // todo: test different values
    final boolean isAutonomous;
    public SpecimenServo(HardwareMap hardwareMap, Gamepad gamepad, boolean isAutonomous) {
        this.gamepad = gamepad;
        this.isAutonomous = isAutonomous;
        rampServo.init(hardwareMap,"specServo"); // port 3
    }

    public void loop() {
        if (/*LinearLift.liftMotor.getCurrentPosition() > 500 &&*/ gamepad.x) {
            rampServo.setServoPosition(OPEN_POSITION);
        } else {
            rampServo.setServoPosition(CLOSED_POSITION);
        }
    }

}

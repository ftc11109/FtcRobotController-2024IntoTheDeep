package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RampServo {

    private final Gamepad gamepad;
    ServoController rampServo  = new ServoController();

    static final double SCORE_POSITION = 0.2;
    static final double LOAD_POSITION = 0.75;
    final boolean isAutonomous;
    public RampServo(HardwareMap hardwareMap, Gamepad gamepad, boolean isAutonomous) {
        this.gamepad = gamepad;
        this.isAutonomous = isAutonomous;
        rampServo.init(hardwareMap,"rampServo"); // port 5
    }

    public void loop() {
        if (/*LinearLift.liftMotor.getCurrentPosition() > 500 &&*/ gamepad.x) {
            rampServo.setServoPosition(SCORE_POSITION);
        } else {
            rampServo.setServoPosition(LOAD_POSITION);
        }
    }

}

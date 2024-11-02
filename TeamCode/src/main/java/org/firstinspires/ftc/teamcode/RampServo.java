package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RampServo {

    private final Gamepad gamepad;
    ServoController rampServo  = new ServoController();
    final boolean isAutonomous;
    public RampServo(HardwareMap hardwareMap, Gamepad gamepad, boolean isAutonomous) {
        this.gamepad = gamepad;
        this.isAutonomous = isAutonomous;
        rampServo.init(hardwareMap,"rampServo"); // port 2
    }

    public void loop() {
        if (gamepad.x) rampServo.setServoPosition(1);
        else rampServo.setServoPosition(0);
    }

}

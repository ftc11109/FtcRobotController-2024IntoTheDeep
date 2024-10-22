package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeServos {

    private final Gamepad gamepad;
    ServoController leftIntakeServo  = new ServoController();
    ServoController rightIntakeServo = new ServoController();

    public IntakeServos(HardwareMap hardwareMap, Gamepad gamepad) {
        this.gamepad = gamepad;
        leftIntakeServo.init(hardwareMap, "intakeServoL");
        rightIntakeServo.init(hardwareMap, "intakeServoR");
    }

    public void loop() {
        if (gamepad.right_trigger > 0) { //intake
            leftIntakeServo.setServoPosition(1.0);
            rightIntakeServo.setServoPosition(-1.0);
        } else if (gamepad.left_trigger > 0) { //ejection
            leftIntakeServo.setServoPosition(-1.0);
            rightIntakeServo.setServoPosition(1.0);
        } else {
            leftIntakeServo.setServoPosition(0);
            rightIntakeServo.setServoPosition(0);
        }
    }

}

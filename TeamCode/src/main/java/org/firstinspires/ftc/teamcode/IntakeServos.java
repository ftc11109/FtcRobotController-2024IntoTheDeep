package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeServos {

    private final Gamepad gamepad;
    ContinuousServoController leftIntakeServo  = new ContinuousServoController();
    ContinuousServoController rightIntakeServo = new ContinuousServoController();

    public IntakeServos(HardwareMap hardwareMap, Gamepad gamepad) {
        this.gamepad = gamepad;
        leftIntakeServo.init(hardwareMap, "intakeServoL");
        rightIntakeServo.init(hardwareMap, "intakeServoR");
        leftIntakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void loop() {
        if (gamepad.right_trigger > 0) { //intake
            leftIntakeServo.setPower(1);
            rightIntakeServo.setPower(1);
        } else if (gamepad.left_trigger > 0) { //ejection
            leftIntakeServo.setPower(-1);
            rightIntakeServo.setPower(-1);
        } else {
            leftIntakeServo.setPower(0);
            rightIntakeServo.setPower(0);
        }
    }

}

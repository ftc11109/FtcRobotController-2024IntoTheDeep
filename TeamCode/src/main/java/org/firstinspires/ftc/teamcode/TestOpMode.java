package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Silly Test OpMode", group="Linear OpMode")
public class TestOpMode extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        IntakeServos intake      = new IntakeServos (hardwareMap, /*      */ gamepad1, false);
        IntakeSlide  intakeSlide = new IntakeSlide  (hardwareMap, telemetry, gamepad2, false);
        IntakeWrist  intakeWrist = new IntakeWrist  (hardwareMap, telemetry, gamepad2, false);
        LinearLift   lift        = new LinearLift   (hardwareMap, telemetry, gamepad2, false);
        Suspension   sus         = new Suspension   (hardwareMap, telemetry, gamepad2, false);

        telemetry.addLine("Servo Test OpMode Initiated");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            intake.loop();
            intakeSlide.loop();
            intakeWrist.loop();
            lift.loop();
            sus.loop();
//            telemetry.addData("Left Servo Position", intake.leftIntakeServo.getServoPosition());
//            telemetry.addData("Right Servo Position", intake.rightIntakeServo.getServoPosition());
        }
    }
}
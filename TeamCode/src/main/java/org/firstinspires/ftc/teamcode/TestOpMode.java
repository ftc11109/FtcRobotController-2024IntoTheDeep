package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Test OpMode", group="Linear OpMode")
public class TestOpMode extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        IntakeServos  intake      = new IntakeServos (hardwareMap, /*      */ gamepad2, false);
        RampServo     ramp        = new RampServo    (hardwareMap, /*      */ gamepad1, false);
        SpecimenServo specimenServo = new SpecimenServo(hardwareMap, gamepad2, false);

        IntakeSlide  intakeSlide = new IntakeSlide  (hardwareMap, telemetry, gamepad2, false);
        IntakeWrist  intakeWrist = new IntakeWrist  (hardwareMap, telemetry, gamepad2, false);
        LinearLift   rampLift    = new LinearLift   (hardwareMap, telemetry, gamepad1, false);
        Suspension   suspension  = new Suspension   (hardwareMap, telemetry, gamepad1, false);

        telemetry.addLine("Test OpMode Initiated");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            intake.loop();
            ramp.loop();
            specimenServo.loop();

            intakeSlide.loop();
            intakeWrist.loop();
            rampLift.newLoop();
            suspension.loop();
//            telemetry.addData("Left Servo Position", intake.leftIntakeServo.getServoPosition());
//            telemetry.addData("Right Servo Position", intake.rightIntakeServo.getServoPosition());
            telemetry.update();
            if (gamepad1.b) {
                SpecimenServo.setIsOpen(false);
                specimenServo.loop();
                sleep(SpecimenServo.ACTUATION_TIME);
                rampLift.setPosition(LinearLift.LOW_BUCKET);
            }
        }
    }
}
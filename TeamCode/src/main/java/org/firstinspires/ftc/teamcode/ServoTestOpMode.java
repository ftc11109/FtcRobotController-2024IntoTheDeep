package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Servo Test OpMode", group="Linear OpMode")
public class ServoTestOpMode extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        IntakeServos intake = new IntakeServos(hardwareMap, gamepad1);

        telemetry.addLine("Servo Test OpMode Initiated");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            intake.loop();
//            telemetry.addData("Left Servo Position", intake.leftIntakeServo.getServoPosition());
//            telemetry.addData("Right Servo Position", intake.rightIntakeServo.getServoPosition());
        }
    }
}
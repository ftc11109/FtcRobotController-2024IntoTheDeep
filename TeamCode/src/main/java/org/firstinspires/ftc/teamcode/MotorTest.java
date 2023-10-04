package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Test: DC Motor \"motor1\"", group="Linear OpMode")
public class MotorTest extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        DcMotor testMotor1 = hardwareMap.get(DcMotor.class, "motor1");
        testMotor1.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // If A is pressed, provide max positive power. If B is pressed, max negative power.
            // If both are pressed, they cancel out.
            double power = 0;
            if (gamepad1.a) {
                power += 1;
            } else if (gamepad1.b) {
                power -= 1;
            }

            testMotor1.setPower(power);

            // Show the elapsed game time and motor power.
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addLine("Press A for forward power. Press B for reverse.");
            telemetry.addData("Power:", "%4.2f", power);
            telemetry.update();
        }
    }
}

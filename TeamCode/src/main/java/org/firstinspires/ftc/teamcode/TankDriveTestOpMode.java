package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/*
 * This file is heavily derived from the following samples; refer back to them for original source:
 *   - samples/BasicOmniOpMode_Linear.java (base class)
 *   - ftc21764/FtcRobotControllerPowerPlay/PowerPlayTeleop.java - for Field-Oriented driving
 *
 * Original Comment:
 *
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Tank Drive Test OpMode", group="Linear OpMode")
public class TankDriveTestOpMode extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        // Note: The names here do not match the hardware names -- this should be fixed in the configuration.
        DcMotor backRightDrive  =  hardwareMap.get(DcMotor.class, "right_driveB");
        DcMotor frontRightDrive =  hardwareMap.get(DcMotor.class, "right_driveF");
        DcMotor frontLeftDrive  =  hardwareMap.get(DcMotor.class, "left_driveF" );
        DcMotor backLeftDrive   =  hardwareMap.get(DcMotor.class, "left_driveB" );

        /*
        Intake intake = new Intake(hardwareMap, telemetry, gamepad1);

        SwingArm swingArm = new SwingArm(hardwareMap, telemetry, gamepad2, false);

        Suspension suspension = new Suspension(hardwareMap, telemetry, gamepad2);

        DoorServo doorServo = new DoorServo(hardwareMap, gamepad2);

        BucketServo bucketServo = new BucketServo(hardwareMap);

        DroneLauncherServo droneLauncherServo = new DroneLauncherServo(hardwareMap, gamepad2);
         */

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.

        frontLeftDrive.setDirection           (DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection            (DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection          (DcMotor.Direction.REVERSE);
        backRightDrive.setDirection           (DcMotor.Direction.FORWARD);
        // due to physical motor orientation, the front two motors must be set to reverse.

        frontLeftDrive.setZeroPowerBehavior   (DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior    (DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior  (DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior   (DcMotor.ZeroPowerBehavior.BRAKE);

        // wait for the game to start (driver presses PLAY)
        // insert silly below
        telemetry.addLine("goober mode successfully initiated :3");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // insert loops here
            // example: intake.loop()

            // controls:
            double y  = -gamepad1.left_stick_y  ;  // note: pushing stick forward gives negative value
            double x  =  gamepad1.left_stick_x  ;
            double rx = -gamepad1.right_stick_x ;

            double botHeading = 0;
            double rotX =  x * Math.cos(botHeading) + y * Math.sin(botHeading);
            double rotY = -x * Math.sin(botHeading) + y * Math.cos(botHeading);

            /*
            comp edited code:
            double rotX =  x * Math.cos(botHeading) + y * Math.sin(botHeading);
            double rotY = -x * Math.sin(botHeading) + y * Math.cos(botHeading);

            default code:
            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);
             */

            // combine the joystick requests for each axis-motion to determine each wheel's power
            // set up a variable for each drive wheel to save the power level for telemetry
            double leftFrontPower  =  y + x + rx;
            double rightFrontPower =  y - x - rx;
            double leftBackPower   =  y - x + rx;
            double rightBackPower  =  y + x - rx;

            // normalize the values so no wheel power exceeds 100%
            // this ensures the robot maintains the desired motion
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Hold the left bumper and the corresponding button to run test code.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            if (gamepad1.left_bumper) {
                leftFrontPower = gamepad1.x ? 1.0 : 0.0;  // X gamepad
                leftBackPower = gamepad1.a ? 1.0 : 0.0;   // A gamepad
                rightFrontPower = gamepad1.y ? 1.0 : 0.0; // Y gamepad
                rightBackPower = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            }

            // send calculated power to wheels
            frontLeftDrive.setPower(leftFrontPower);
            frontRightDrive.setPower(rightFrontPower);
            backLeftDrive.setPower(leftBackPower);
            backRightDrive.setPower(rightBackPower);

            // display elapsed game time, wheel power, misc. debug utils
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addLine("");

            telemetry.addData("IMU orientation", botHeading);
            telemetry.addData("rotX, rotY", "%4.2f, %4.2f", rotX, rotY);
            telemetry.addLine("");

            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower,  rightBackPower );
            telemetry.addLine("");

            telemetry.addLine("LB + A/B/X/Y to test single motors");
            telemetry.update();
        }
    }
}
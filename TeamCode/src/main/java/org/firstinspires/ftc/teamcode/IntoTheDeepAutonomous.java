package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Objects;

/*
 *  This file illustrates the concept of driving an autonomous path based on Gyro heading and encoder counts.
 *  The code is structured as a LinearOpMode
 *
 *  The path to be followed by the robot is built from a series of drive, turn or pause steps.
 *  Each step on the path is defined by a single function call, and these can be strung together in any order.
 *
 *  The code REQUIRES that you have encoders on the drive motors, otherwise you should use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that you have a BOSCH BNO055 IMU, otherwise you would use: RobotAutoDriveByEncoder;
 *  This IMU is found in REV Control/Expansion Hubs shipped prior to July 2022, and possibly also on later models.
 *  To run as written, the Control/Expansion hub should be mounted horizontally on a flat part of the robot chassis.
 *
 *  This sample requires that the drive Motors have been configured with names : left_drive and right_drive.
 *  It also requires that a positive power command moves both motors forward, and causes the encoders to count UP.
 *  So please verify that both of your motors move the robot forward on the first move.  If not, make the required correction.
 *  See the beginning of runOpMode() to set the FORWARD/REVERSE option for each motor.
 *
 *  This code uses RUN_TO_POSITION mode for driving straight, and RUN_USING_ENCODER mode for turning and holding.
 *  Note: You must call setTargetPosition() at least once before switching to RUN_TO_POSITION mode.
 *
 *  Notes:
 *
 *  All angles are referenced to the coordinate-frame that is set whenever resetHeading() is called.
 *  In this sample, the heading is reset when the Start button is touched on the Driver station.
 *  Note: It would be possible to reset the heading after each move, but this would accumulate steering errors.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clockwise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 *
 *  Control Approach.
 *
 *  To reach, or maintain a required heading, this code implements a basic Proportional Controller where:
 *
 *      Steering power = Heading Error * Proportional Gain.
 *
 *      "Heading Error" is calculated by taking the difference between the desired heading and the actual heading,
 *      and then "normalizing" it by converting it to a value in the +/- 180 degree range.
 *
 *      "Proportional Gain" is a constant that YOU choose to set the "strength" of the steering response.
 *
 *  Use Android Studio to Copy this Class, and Paste it into your "TeamCode" folder with a new name.
 *  Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Into The Autonomous", group="Robot")
//@Disabled
public class IntoTheDeepAutonomous extends LinearOpMode {

    /* Declare OpMode members. */
    protected DcMotor frontLeftDrive = null;
    protected DcMotor backLeftDrive = null;
    protected DcMotor frontRightDrive = null;
    protected DcMotor backRightDrive = null;
    protected IMU imu = null;

    protected IntakeServos intake;
    protected RampServo rampServo;
    protected IntakeSlide intakeSlide;
    protected IntakeWrist intakeWrist;
    protected LinearLift rampLift;

    //protected ElapsedTime runtime = new ElapsedTime();

    private double robotHeading = 0;
    private double headingOffset = 0;
    private double headingError = 0;


    // These variables are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double targetHeading = 0;
    private double turnSpeed = 0;
    private double leftSpeed = 0;
    private double rightSpeed = 0;

    int driveStraightLoops = 0;

    double tbegin;

    //private AprilTagProcessor tagProcessor;
    //private VisionPortal tagsVisionPortal; what? no, this isn't code, what are you talking about?

    double MMperIN = 25.4;
    int wheelDiaMM = 75;
    double wheelDiaIN = wheelDiaMM / MMperIN; //or input just inches as constant
    double wheelCircum = wheelDiaIN * Math.PI; //get circum (aka inches per wheel rev)
    int ultPlanHexEncoderTicks = 28; //ticks per motor rotation
    double threeToOne = 84.0 / 29.0; // real 3:1
    double fourToOne = 76.0 / 21.0; // real 4:1
    double drivetrainMotorGearRatio = threeToOne * fourToOne; //get gear ratio

    public double inchesPerTick() {
        return (wheelCircum / (drivetrainMotorGearRatio * ultPlanHexEncoderTicks)); //Inches per tick
        //return ((drivetrainMotorGearRatio * ultPlanHexEncoderTicks)/wheelCircum) * inches; //Ticks per inch
    }

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double DRIVE_SPEED = 0.45;     // Max driving speed for better distance accuracy.
    //static final double SLOW_DRIVE_SPEED = 0.15;
    //static final double FAST_DRIVE_SPEED = 0.6; h
    static final double TURN_SPEED = 0.45;     // Max Turn speed to limit turn rate
    //static final double SCORE_DRIVE_SPEED = 0.15;
    //static final double SLOW_TURN_SPEED = 0.15;
    //static final double FAST_TURN_SPEED = 0.6; h
    static final double HEADING_THRESHOLD = 4.0;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double P_TURN_GAIN = 0.02;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_GAIN = 0.03;     // Larger is more responsive, but also less stable

    //int startingTile = 0; // which tile our robot starts on. 0 is cornered in the observation zone's triangle. OBSOLETE

    //this sets up for bulk reads!
    protected List<LynxModule> allHubs;

    protected void setupRobot() {
        // Initialize the drive system variables.
        backLeftDrive   = hardwareMap.get(DcMotor.class, "left_driveB" );
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "left_driveF" );
        backRightDrive  = hardwareMap.get(DcMotor.class, "right_driveB");
        frontRightDrive = hardwareMap.get(DcMotor.class, "right_driveF");

        intake          = new IntakeServos(hardwareMap, gamepad2, true);
        rampServo       = new RampServo   (hardwareMap, gamepad2, true);

        intakeSlide     = new IntakeSlide (hardwareMap, telemetry, gamepad2, true);
        intakeWrist     = new IntakeWrist (hardwareMap, telemetry, gamepad2, true);
        rampLift        = new LinearLift  (hardwareMap, telemetry, gamepad2, true);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        frontLeftDrive.setDirection           (DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection            (DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection          (DcMotor.Direction.REVERSE);
        backRightDrive.setDirection           (DcMotor.Direction.FORWARD);

        // TODO: Figure out if it's better to use a static variable for imu
        // and then avoid re-initializing it if you're in teleop mode and it already exists.
        // That way the current orientation is not reset when the opmode starts.

        // define initialization values for IMU, and then initialize it.

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )));

        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        allHubs = hardwareMap.getAll(LynxModule.class);

        //sets up for bulk reads in manual mode! Read about it here: https://gm0.org/en/latest/docs/software/tutorials/bulk-reads.html
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // By default the value is 250ms... we send data to Driver Station 4x per second.
        // We can see if our loop runs faster if we essentially disable telemetry by putting
        // a high number here.
        // Change this so that we keep telemetry on during init but disable it during run mode
        //telemetry.setMsTransmissionInterval(10000;)
    }

    /**
     * Checks on linear slide, four bar, and intake inside driving loops so that they can update themselves
     */
    protected void mechanismLoop() {
        intakeSlide.loop();
        intakeWrist.loop();
        rampLift.loop();
        rampServo.loop( );
    }

    @Override
    public void runOpMode() {
        setupRobot();
        // Wait for the game to start (Display Gyro value while waiting)
        while (opModeInInit()) {

            /*
            if (gamepad1.dpad_right && startingTile > 0) {
                startingTile -= 1;
                while (gamepad1.dpad_right) STFU();
            }
            if (gamepad1.dpad_left && startingTile < 3) {
                startingTile += 1;
                while (gamepad1.dpad_left) STFU();
            }
            telemetry.addData("starting tile", startingTile);
            //telemetry.addData("", "Robot Heading = %4.0f", getRawHeading());
            telemetry.addData("bot heading:", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

            above is setup code from test auto */



            telemetry.addData("left front starting:", frontLeftDrive.getCurrentPosition());
            telemetry.addData("left back starting:", backLeftDrive.getCurrentPosition());
            telemetry.addData("right front starting:", frontRightDrive.getCurrentPosition());
            telemetry.addData("right back starting:", backRightDrive.getCurrentPosition());
            telemetry.addLine("");

            //telemetry.addData();

            telemetry.update();


        }
        tbegin = (double) getRuntime();

        // Set the encoders for closed loop speed control, and reset the heading.
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetHeading();

        runAutonomousProgram();

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
        // Pause to display last telemetry message.
    }


    public void runAutonomousProgram() {

        /*double distanceIN_observationZone_netZone = 94.5; h

        driveStraight(DRIVE_SPEED, 8, 0);
        turnToHeading(TURN_SPEED, 90);
        driveStraight(DRIVE_SPEED, distanceIN_observationZone_netZone - (startingTile * 23.6), 90);
        turnToHeading(TURN_SPEED, -45);

        //score
        lift.setPosition(LinearLift.HIGH_BUCKET);
        rampScore();
        lift.setPosition(LinearLift.LOW_HARDSTOP);

        //return to
        turnToHeading(TURN_SPEED, -90);
        driveStraight(DRIVE_SPEED, distanceIN_observationZone_netZone, -90);*/

        /*driveStraight(DRIVE_SPEED, 6, -45);
        turnToHeading(TURN_SPEED, 0); h */


        //eat sample
        /*intakeWrist.setPosition(IntakeWrist.DEPLOYED_POSITION);
        runIntake(1, 1250);
        intakeWrist.setPosition(IntakeWrist.TRANSFER_POSITION);
        runIntake(-1, 1250);
        h */
        /*//Move away from the starting point and turn towards the net zone.
        driveStraight(DRIVE_SPEED, 6, 0);
        turnToHeading(TURN_SPEED, -90);
        //Lift the slide, (and the ramp (and the sample in the ramp))
        rampLift.setPosition(LinearLift.HIGH_BUCKET); //it does NOT need +40 anymore. smh my head
        //Move towards the net zone and turn.
        driveStraight(DRIVE_SPEED, -46, -90);
        turnToHeading(TURN_SPEED, -45);

        //rampLift.runToAndWait(LinearLift.HIGH_BUCKET);

        //while(!rampLift.isAtTarget()) {}
        //Slight delay to avoid shaking messing up our really nice auto
        //sleep(300);
        //Then tilt the servo (and the bucket (and the sample)) making the sample fall into the bucket.
        rampScore();
        // Then lower arm, intake sample, move arm (and sample) up and eject sample. Then move the linear
        // lift (and ramp (and the sample)) to score position and score. While that's gong, arm goes down and intakes
        // sample and moves back up with sample and waits for ramp to go back down and ejects it and repeats two times
        // (three if we can) and go to ascent zone to perform a level 1 ascension
        rampLift.setPosition(LinearLift.LOW_HARDSTOP);*/

        // PHASE 0: drive to net zone
        driveStraight(DRIVE_SPEED, 6, 0);
        turnToHeading(TURN_SPEED, -90);

        rampLift.setPosition(LinearLift.HIGH_BUCKET);
        driveStraight(DRIVE_SPEED, -14.5, -90);

        // PHASE 1: score preloaded sample
        turnToHeading(TURN_SPEED, -45);
        driveStraight(DRIVE_SPEED, -3, -45);
        while (!rampLift.isAtTarget() && opModeIsActive()) {
            rampLift.loop();
            telemetry.update();
        }
        rampScore();

        rampLift.setPosition(LinearLift.LOW_HARDSTOP);

        // PHASE 2: eat another sample off the ground
        // repeat this to score multiple samples. change parameters as necessary
        turnToHeading(TURN_SPEED, 0);
        driveStraight(DRIVE_SPEED, 5, 0);

        intakeWrist.setPosition(IntakeWrist.DEPLOYED_POSITION);
        while (!intakeWrist.isAtTarget(10) && opModeIsActive()) {
            intakeWrist.loop();
            rampLift.loop();
            telemetry.update();
        }
//skibidi
        runIntake(1, 3000);

        intakeWrist.setPosition(IntakeWrist.TRANSFER_POSITION);
        while (!intakeWrist.isAtTarget(10) && opModeIsActive()) {
            intakeWrist.loop();
            telemetry.update();
        }
        runIntake(-1, 1000);

            // return to net zone
        driveStraight(DRIVE_SPEED, -5, 0);
        turnToHeading(TURN_SPEED, -45);

            // score
        rampLift.setPosition(LinearLift.HIGH_BUCKET);
        rampScore();
        rampLift.setPosition(LinearLift.LOW_HARDSTOP);

        turnToHeading(TURN_SPEED, 0);


        /*

        todo: add greater tolerance to driveStraight, so it doesn't pause for too long after a movement

        BASIC AUTO:

        Preload a sample
        Drive to Bucket Zone
        Score sample in high basket
        Commit level 1 ascension


         * AUTO 1
         * Preload a specimen
         * Parameters: Starting square, Ascent square,
         * Scores preloaded specimen in low chamber. (we cant reach the high one)
         * Moves to the middle of spike marks and turns 2 (3 if we have time) samples into specimens.
         * Scores them on low chamber and preforms level 1 ascent
         */

    }

    /*
     * ====================================================================================================
     * Driving "Helper" functions are below this line.
     * These provide the high and low level methods that handle driving straight and turning.
     * ====================================================================================================
     */

    // **********  HIGH Level driving functions.  ********************

    /**
     * Method to drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param distance      Distance (in inches) to move from current position.  Negative distance means move backward.
     * @param heading       Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                      0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                      If a relative angle is required, add/subtract from the current robotHeading.
     */
    public void driveStraight(double  maxDriveSpeed,
                              double  distance,
                              double  heading) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {



            //reverse the heading if you start on the left side. this turns a right heading into a left heading and vice versa.
            //heading = heading * reverseTurnsForAllianceColor;

            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (distance / inchesPerTick()); //total amount of encoder ticks between the current position and the destination

            int leftTargetF = frontLeftDrive.getCurrentPosition() + moveCounts;
            int leftTargetB = backLeftDrive.getCurrentPosition() + moveCounts;
            int rightTargetF = frontRightDrive.getCurrentPosition() + moveCounts;
            int rightTargetB = backRightDrive.getCurrentPosition() + moveCounts;


            /*telemetry.addData("left front moved:", frontLeftDrive.getCurrentPosition());
            telemetry.addData("left back moved:", backLeftDrive.getCurrentPosition());
            telemetry.addData("right front moved:", frontRightDrive.getCurrentPosition());
            telemetry.addData("right back moved:", backRightDrive.getCurrentPosition()); h */


            frontLeftDrive.setTargetPosition(leftTargetF);
            backLeftDrive.setTargetPosition(leftTargetB);
            frontRightDrive.setTargetPosition(rightTargetF);
            backRightDrive.setTargetPosition(rightTargetB);

            /*telemetry.addData("driveStraight", "opModeIsActive");
            telemetry.addData("targetPositions", "%d : %d : %d : %d", leftTargetF, leftTargetB, rightTargetF, rightTargetB);
            telemetry.addData("move counts:", moveCounts);
            telemetry.update(); h */

            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            /*telemetry.addData("maxDriveSpeed", maxDriveSpeed);
            telemetry.addData("active", opModeIsActive());
            telemetry.addData("ldf", frontLeftDrive.isBusy());
            telemetry.addData("rdf", frontRightDrive.isBusy());
            telemetry.addData("ldb", backLeftDrive.isBusy());
            telemetry.addData("rdb", backRightDrive.isBusy()); h */
            // Unfortunately we need this because sometimes the motor hasn't recognized yet that it's busy!!
            //sleep(1000);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);
            driveStraightLoops += 1;

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive()) {

                /*telemetry.addData("driveStraight", "opModeIsActive and all motors are busy!");
                telemetry.addData("drive straight loops: ", driveStraightLoops); h */

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(maxDriveSpeed, turnSpeed);

                mechanismLoop();

                // Display drive status for the driver.
                //sendTelemetry(true);

                telemetry.addData("LeftSpeed", leftSpeed);
                telemetry.addData("RightSpeed", rightSpeed);

                telemetry.addData("Steering Correction", getSteeringCorrection(heading, P_DRIVE_GAIN));
                telemetry.addData("IMU", "%4.2f", getRawHeading());
                telemetry.addData("robotHeading", "%4.2f", robotHeading);
                telemetry.addData("headingError", headingError);

                telemetry.addData("target heading", targetHeading);


                telemetry.update();


                clearBulkCache();

                // Check if ALL motors report not busy
                if (!(frontLeftDrive.isBusy() || frontRightDrive.isBusy() || backLeftDrive.isBusy() || backRightDrive.isBusy())) {
                    break;
                }


            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                     0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                     If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            mechanismLoop();

            // Display drive status for the driver.
            //sendTelemetry(false);

            clearBulkCache();
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     * This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     * @param maxTurnSpeed Maximum differential turn speed (range 0 to +1.0)
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                     0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                     If a relative angle is required, add/subtract from current heading.
     * @param holdTime     Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            mechanismLoop();

            // Display drive status for the driver.
            //sendTelemetry(false);

            telemetry.addData("LeftSpeed" , leftSpeed );
            telemetry.addData("RightSpeed", rightSpeed);

            telemetry.update();

            clearBulkCache();

        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    // **********  LOW Level driving functions.  ********************
    /**
     * This method uses a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading   The desired absolute heading (relative to last heading reset)
     * @param proportionalGain Gain factor applied to heading error to obtain turning power.
     * @return Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError >   180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * This method takes separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     *
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn) {
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed  = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed  /= max;
            rightSpeed /= max;
        }

        frontLeftDrive.setPower(leftSpeed);
        backLeftDrive.setPower(leftSpeed);
        frontRightDrive.setPower(rightSpeed);
        backRightDrive.setPower(rightSpeed);
    }

    public void strafeMoveRobot(String direction, double drive, double turn) {
        //driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        //turnSpeed = turn;  h   // save this value as a class member so it can be used by telemetry.

        double frontSpeed = drive + turn;
        double backSpeed = drive - turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        if (Objects.equals(direction, "left")) {
            frontLeftDrive.setPower(-frontSpeed);
            backLeftDrive.setPower(backSpeed);
            frontRightDrive.setPower(frontSpeed);
            backRightDrive.setPower(-backSpeed);
        } else {
            frontLeftDrive.setPower(frontSpeed);
            backLeftDrive.setPower(-backSpeed);
            frontRightDrive.setPower(-frontSpeed);
            backRightDrive.setPower(backSpeed);
        }
    }

    public boolean closeEnough(int tickThreshold) {
        return
                (Math.abs(frontLeftDrive .getCurrentPosition() - frontLeftDrive .getTargetPosition()) >= tickThreshold) &&
                (Math.abs(backLeftDrive  .getCurrentPosition() - backLeftDrive  .getTargetPosition()) >= tickThreshold) &&
                (Math.abs(frontRightDrive.getCurrentPosition() - frontRightDrive.getTargetPosition()) >= tickThreshold) &&
                (Math.abs(backRightDrive .getCurrentPosition() - backRightDrive .getTargetPosition()) >= tickThreshold) ;
    }

    protected void clearBulkCache() {
        // Clears the cache so that it will be refreshed the next time you ask for a sensor value!
        // Be very sure that this gets called in every loop in your code!!!!
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    /*
     * Display the various control parameters while driving
     *
     * @param straight Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    /*private void sendTelemetry(boolean straight) { h //SHOULD BE LABELED SOMETHING LIKE "driveSendTelemetry"
        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos LF:RF:LB:RB", "%7d:%7d:%7d:%7d",
                    leftTargetF, rightTargetF, leftTargetB, rightTargetB);
            telemetry.addData("Actual Pos LF:RF:LB:RB", "%7d:%7d:%7d:%7d", frontLeftDrive.getCurrentPosition(),
                    frontRightDrive.getCurrentPosition(), backLeftDrive.getCurrentPosition(), backRightDrive.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
        telemetry.addData("Error:Steer", "%5.1f:%5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L:R.", "%5.2f : %5.2f", leftSpeed, rightSpeed);

        telemetry.addData("drive straight loops: ", driveStraightLoops);
        telemetry.addData("current driveSpeed value: ", driveSpeed);

        //checks the time spent on the loop and adds it to telemetry

        telemetry.addData("Loop Time", (int) runtime.milliseconds());

        runtime.reset();

        telemetry.update();
    }*/

    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    public double getRawHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    /**
     * Reset the "offset" heading back to zero
     */
    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }

    public void runIntake(int power, long time) {

        intake.leftIntakeServo.setPower(power);
        intake.rightIntakeServo.setPower(power);
        sleep(time);
        intake.leftIntakeServo.setPower(0);
        intake.rightIntakeServo.setPower(0);

    }

    public void rampScore() {
        rampServo.rampServo.setServoPosition(RampServo.SCORE_POSITION);
        sleep(750);
        rampServo.rampServo.setServoPosition(RampServo.LOAD_POSITION);
        sleep(750);
    }

    /**
     * Makes empty while() statements stop throwing warnings (yapping).
     * Add this inside of your empty while() loop for a "wait until" sort of function, like in Scratch.
     */
    public void doNothing() {}
}
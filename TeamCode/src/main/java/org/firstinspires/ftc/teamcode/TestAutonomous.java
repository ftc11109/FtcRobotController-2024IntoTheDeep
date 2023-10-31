/** -Pseudocode Notes-

Encoders:
UltraPlanetary motor: 28 cpr (counts per rotation)
Core Hex motor encoder: 4 cpr @ motor, 288 cpr @ output

Ratios:
3:1 real - 84:29
4:1 real - 76:21
5:1 real - 68:13

Math for distance function:
Wheel diameter (mm) * PI / Gear ratio / Encoder ticks * mm. to in.

Functions:
turn(int deg) must turn the robot deg degrees. It should have a variability value of about 1 degree.
move(String dir, double dist) moves the robot dist inches. It should account for speed-up and ramp-down of the motors.

constants:

MMperIN = 25.4
wheelDiaMM = 75
wheelDiaIN = wheelDiaMM / MMperIN //or input just inches as constant
wheelCircum = wheelDiaIN * Pi() //import math lib later
ultPlanHexEncoderTicks = 28
3to1 = 84/29 // real 3:1
4to1 = 76/21 // real 4:1
drivetrainMotorGearRatio = 3to1 * 4to1

public double inchesToTicks(inches) {
    (wheelCircum/(drivetrainMotorGearRatio * ultPlanHexEncoderTicks)) * inches
}



 */
package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="CenterStage Auto Test", group="Robot")
//@Disabled
public class TestAutonomous extends LinearOpMode {

    boolean isNear;
    boolean parkLeft;
    Intake intake = new Intake();
    private DcMotor leftDriveFront = null;
    private DcMotor rightDriveFront = null;
    private DcMotor leftDriveBack = null;
    private DcMotor rightDriveBack = null;

    private int     leftTargetF    = 0;
    private int     leftTargetB    = 0;
    private int     rightTargetF   = 0;
    private int     rightTargetB   = 0;
    private double  driveSpeed     = 0;
    private double  turnSpeed      = 0;
    private boolean leftDrive      = false;
    private boolean rightDrive     = false;

    int moveCounts = 0;

    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    // These values are from the sample code, we can change them later

    //    void turn(String dir, int time) {
//    //Make turning rely on absolute cardinal direction instead of turn time (later)
//        if (dir == "l") {
//            for (int i = 0; i < time; i++) {
//                leftDriveFront.setPower(-TURN_SPEED);
//                leftDriveBack.setPower(-TURN_SPEED);
//                rightDriveFront.setPower(TURN_SPEED);
//                rightDriveBack.setPower(TURN_SPEED);
//            }
//            leftDriveFront.setPower(0);
//            leftDriveBack.setPower(0);
//            rightDriveFront.setPower(0);
//            rightDriveBack.setPower(0);
//        }
//        else {
//            for (int i = 0; i < time; i++) {
//                leftDrive.setPower(TURN_SPEED);
//                rightDrive.setPower(-TURN_SPEED);
//            }
//            leftDrive.setPower(0);
//            rightDrive.setPower(0);
//        }
//    }
    final double MMperIN = 25.4;
    final int wheelDiaMM = 75;
    final double wheelDiaIN = wheelDiaMM / MMperIN; //or input just inches as constant
    final double wheelCircum = wheelDiaIN * Math.PI;
    final int ultPlanHexEncoderTicks = 28; //ticks per motor rotation
    final double threeToOne = 84f/29f; // real 3:1
    final double fourToOne = 76f/21f; // real 4:1
    final double driveTrainGearRatio = threeToOne * fourToOne; //get gear ratio

    public double inchesPerTick() {
        return (wheelCircum/(driveTrainGearRatio * ultPlanHexEncoderTicks)); //Inches per tick
        //return ((drivetrainMotorGearRatio * ultPlanHexEncoderTicks)/wheelCircum) * inches; //Ticks per inch
    }

    //define target motor positions by adding ticks to go to absolute tick position
    void setStraightTarget(int moveCounts) {
        leftTargetF = leftDriveFront.getCurrentPosition() + moveCounts;
        leftTargetB = leftDriveBack.getCurrentPosition() + moveCounts;
        rightTargetF = rightDriveFront.getCurrentPosition() + moveCounts;
        rightTargetB = rightDriveBack.getCurrentPosition() + moveCounts;
        //set targets
        leftDriveFront.setTargetPosition(leftTargetF);
        leftDriveBack.setTargetPosition(leftTargetB);
        rightDriveFront.setTargetPosition(rightTargetF);
        rightDriveBack.setTargetPosition(rightTargetB);
    }

    void moveRobot(double drive_speed, int direction) {

        if (direction > 0) {
            leftDrive = false;
            rightDrive = true;
        } else if (direction < 0) {
            leftDrive = true;
            rightDrive = false;
        } else {
            leftDrive = true;
            rightDrive = true;
        }

        //TODO: add dampening for drive_speed
        leftDriveFront.setPower  (drive_speed * MiscFunc.boolToInt(leftDrive));
        leftDriveBack.setPower   (drive_speed * MiscFunc.boolToInt(leftDrive));
        rightDriveFront.setPower (drive_speed * MiscFunc.boolToInt(rightDrive));
        rightDriveBack.setPower  (drive_speed * MiscFunc.boolToInt(rightDrive));
    }


    void driveToTarget() {
        //run to targets
        leftDriveFront.setMode  (DcMotor.RunMode.RUN_TO_POSITION);
        leftDriveBack.setMode   (DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveFront.setMode (DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveBack.setMode  (DcMotor.RunMode.RUN_TO_POSITION);
        moveRobot(0.5, 0);
    }

    void driveToDistance(double drive_speed, int direction, double inches_to_move) {

        if (opModeIsActive()) {

            double ticksToGo = inches_to_move / inchesPerTick(); //Get amount of ticks to go for inches to move using helper function

            int moveCounts = (int) (ticksToGo) * direction; //Convert to inches

            moveRobot(driveSpeed, direction); //Set motor speed and direction to defined value
            setStraightTarget(moveCounts); //Pass ticks for distance to move through target function
            driveToTarget(); //Drive motors to encoder target

            while (opModeIsActive()) {

                //Course-correction
                moveRobot(driveSpeed,direction);

                //Detects if any motor stops being busy
                if (
                    !(  leftDriveFront.isBusy()  ||
                        rightDriveFront.isBusy() ||
                        leftDriveBack.isBusy()   ||
                        rightDriveBack.isBusy()  )
                ) {
                    //Ends course-correction
                    moveRobot(0, 0);

                    //Ends loop
                    break;

                }
            }
        }
    }
    public void runOpMode() {

        //TODO: Inject more Groovy

        while (opModeInInit()) {
            telemetry.addLine("Waiting...");
            telemetry.update();
        }
        leftDriveFront  = hardwareMap.get(DcMotor.class, "left_driveF");
        rightDriveFront = hardwareMap.get(DcMotor.class, "right_driveF");
        leftDriveBack   = hardwareMap.get(DcMotor.class, "left_driveB");
        rightDriveBack  = hardwareMap.get(DcMotor.class, "right_driveB");

        leftDriveFront.setDirection   (DcMotor.Direction.FORWARD);
        leftDriveBack.setDirection    (DcMotor.Direction.FORWARD);
        rightDriveFront.setDirection  (DcMotor.Direction.FORWARD);
        rightDriveBack.setDirection   (DcMotor.Direction.REVERSE);
        //Fixed motor directions (21764 had reversed motor) - this would be changed back to normal this year's robot
//        leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        driveToDistance(0.5, 1, 48);
        telemetry.addLine("Path complete"); // Used to look like this: telemetry.addData("Path", "complete"); tf???
        telemetry.update();
        leftDriveFront.setMode  (DcMotor.RunMode.RUN_USING_ENCODER);
        leftDriveBack.setMode   (DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveFront.setMode (DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveBack.setMode  (DcMotor.RunMode.RUN_USING_ENCODER);
        //Step 1: detect the team prop
        SpikeMark teamPropMark = detectTeamProp();
        //Step 2: drive to the team prop
        driveToCorrectSpikeMark(teamPropMark);
        //Step 3: place purple pixel on same line as team prop
        ejectPurplePixel();
        //Step 4: return to original position
        driveFromSpikeMark(teamPropMark);
        //Step 5: drive under truss closest to wall to get to backdrop
        driveToBackdrop(isNear);
        //Step 6: place yellow pixel on correct April tag
        depositYellowPixel(teamPropMark);
        //Step 7: drive off to side
        parkInBackstage(parkLeft);
    }

    enum SpikeMark {RIGHT, LEFT, CENTER}
    SpikeMark detectTeamProp(){
        return SpikeMark.CENTER;
    }
    void driveForwardIn(double inches) {
        //
    }
    void turnDegrees(double degrees) {
        //finish this later
    }
    final double CENTER_SPIKE_MARK_DIST_IN = -1 /* placeholder value */ ;
    final double OFFCENTER_SPIKE_MARK_DIST_IN = -2 /* placeholder value */ ;
    final double SPIKE_MARK_DECISION_DIST = -0.5 /* placeholder value */ ;
    final double OFFCENTER_DECISION_TURN_DEG = -45 /* placeholder value */ ;
    void driveToCorrectSpikeMark(SpikeMark teamPropMark) {
        driveForwardIn(SPIKE_MARK_DECISION_DIST);
//switch to case statement later
        if(teamPropMark == SpikeMark.CENTER) {
            driveForwardIn(CENTER_SPIKE_MARK_DIST_IN);
        } else if (teamPropMark == SpikeMark.LEFT) {
            turnDegrees(OFFCENTER_DECISION_TURN_DEG);
            driveForwardIn(OFFCENTER_SPIKE_MARK_DIST_IN);
        } else if (teamPropMark == SpikeMark.RIGHT) {
            turnDegrees(-OFFCENTER_DECISION_TURN_DEG);
            driveForwardIn(OFFCENTER_SPIKE_MARK_DIST_IN);
        }
    }
    void ejectPurplePixel() {
        intake.ejectPixel();
    }

    void driveFromSpikeMark(SpikeMark teamPropMark){
        //switch to case statement later
        if(teamPropMark == SpikeMark.CENTER) {
            driveForwardIn(-CENTER_SPIKE_MARK_DIST_IN);
        } else if (teamPropMark == SpikeMark.LEFT) {
            turnDegrees(-OFFCENTER_DECISION_TURN_DEG);
            driveForwardIn(-OFFCENTER_SPIKE_MARK_DIST_IN);
        } else if (teamPropMark == SpikeMark.RIGHT) {
            turnDegrees(OFFCENTER_DECISION_TURN_DEG);
            driveForwardIn(-OFFCENTER_SPIKE_MARK_DIST_IN);
        }
    }

    void driveToBackdrop(boolean isNear){}

    void depositYellowPixel(SpikeMark teamPropMark){}

    void parkInBackstage(boolean parkLeft){}


}
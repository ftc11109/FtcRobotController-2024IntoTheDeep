package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LinearLift {

    protected static DcMotor liftMotor;
    private final Telemetry telemetry;
    private final Gamepad gamepad;


    private final ElapsedTime runtime = new ElapsedTime();

    static final int LOW_HARDSTOP = 3;
    static final int HIGH_HARDSTOP = 2600;
    //limit: 3017 - 3011
    static final int HIGH_BUCKET = 2525;
    static final int LOW_BUCKET = 1710;

    static final double MAX_SPEED = 0.85;

    static final int LOW_ERROR_MARGIN = 5;
    static final int HIGH_ERROR_MARGIN = 20;

    final boolean isAutonomous;

    public int targetPositionCount;

    // todo: determine what speed to set the motors to & whether up speed is different than down speed

    public LinearLift(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad, boolean isAutonomous) {
        // if linear slide doesn't run in auto, isAutonomous will be unnecessary

        liftMotor = hardwareMap.get(DcMotor.class,"liftMotor"); // port 2

        this.isAutonomous = isAutonomous;
        this.gamepad = gamepad;
        this.telemetry = telemetry;

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // todo: figure out which value is best
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setTargetPosition(LOW_HARDSTOP);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(0);

        targetPositionCount = LOW_HARDSTOP;

        telemetry.addData("Lift motor position", liftMotor.getCurrentPosition());

    }

    /**
     * utilizes DcMotor.Runmode.RUN_TO_POSITION to set the motor's target
     * @param ticks the distance the motor must run to, measured in ticks
     */
    public void setPosition(int ticks) {
        targetPositionCount = ticks;
        liftMotor.setTargetPosition(ticks);
    }

    private void readGamepad(Gamepad gamepad) {

        /*if (gamepad.left_stick_y > 0.1 || gamepad.left_stick_y < -0.1 ) {

            targetPositionCount = Range.clip((int)(targetPositionCount + ADJUSTMENT_MODIFIER*-gamepad.left_stick_y), LOW_HARDSTOP, HIGH_HARDSTOP);

            telemetry.addData("Manual Branch", "Adjustment made");

        } else if (!liftMotor.isBusy()) {

            //This is so that if you let go of the joystick, it immediately stops the arm from moving. Not a bug!!!

            targetPositionCount = Range.clip(liftMotor.getCurrentPosition(), LOW_HARDSTOP, HIGH_HARDSTOP);
            liftMotor.setTargetPosition(targetPositionCount);

            telemetry.addData("Manual Branch", "Stop moving");
        } else {
            telemetry.addData("Manual Branch", "Running to Junction");

        }

        telemetry.addData("target position count", targetPositionCount);
        telemetry.addData("additive math", ADJUSTMENT_MODIFIER*-gamepad.left_stick_y);*/

        if (gamepad.a) targetPositionCount = LOW_HARDSTOP;
        if (gamepad.b) targetPositionCount = LOW_BUCKET;
        if (gamepad.y) targetPositionCount = HIGH_BUCKET;

    }

    public void loop() {
        if (!isAutonomous) readGamepad(gamepad);
        liftMotor.setTargetPosition(targetPositionCount);

        if (targetPositionCount == LOW_HARDSTOP && isAtTarget()) {
            liftMotor.setPower(0);
            //liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } else {
            if (isAtTarget()) {
                liftMotor.setPower(0.01);
            } else {
                liftMotor.setPower(MAX_SPEED); // this should hopefully stop our lift from falling
            }
        }

        /*

        todo: recalibrate the lift once it gets to the low hardstop. whenever we set the lift to
         max extension and bring it back down, its resting position(at the hardstop) keeps
         increasing. this could be a hardware issue, but this should be implemented as a safety
         protocol regardless.

        for now, we are simply increasing the linear slide's margin for error.

           */

        telemetry.addData("Lift encoder position", liftMotor.getCurrentPosition());
        telemetry.addData("Lift power", liftMotor.getPower());
        telemetry.update(); // test
    }

    public void newLoop() {
        if (!isAutonomous) readGamepad(gamepad);
        liftMotor.setTargetPosition(targetPositionCount);

        /*
        if (targetPositionCount == LOW_HARDSTOP && isAtTarget(10)) {
            liftMotor.setPower(0);
            //liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } else {
            if (isAtTarget()) {
                liftMotor.setPower(0.01);
            } else {
                liftMotor.setPower(MAX_SPEED); // this should hopefully stop our lift from falling
            }
        }
        */

        /* 3 conditions:
         * if at low hardstop,
         *  set power 0
         *  stop and reset encoder
         *
         *

         */

        if (liftMotor.getMode() != DcMotor.RunMode.STOP_AND_RESET_ENCODER)

        telemetry.addData("Lift encoder position", liftMotor.getCurrentPosition());
        telemetry.addData("Lift power", liftMotor.getPower());
        telemetry.update(); // test
    }
    public int getPosition() {
        return liftMotor.getCurrentPosition();
    }

    /**
     * This method is used to set the encoder target of a motor (in this case, our {@linkplain LinearLift#liftMotor}).
     * Uses {@link DcMotor#setTargetPosition(int)} to set the target position of the motor,
     * and updates {@linkplain LinearLift#targetPositionCount} for consistency and for telemetry purposes.
     * <p> <em> Motor must be set to {@linkplain DcMotor.RunMode#RUN_TO_POSITION}. </em>
     * <p>Called inside of an empty while loop like this:
     * <pre> while(runToAndWait); </pre>
     * <p> usage of doNothing() inside of empty while loops not covered by warranty.
     * @param ticks Sets the desired encoder position for the motor to run to.
     */
    public boolean runToAndWait(int ticks) {
        targetPositionCount = ticks;
        liftMotor.setTargetPosition(ticks);
        return !isAtTarget();
    }
    public boolean isAtTarget(int tickThreshold) {
        return Math.abs(liftMotor.getCurrentPosition() - targetPositionCount) <= tickThreshold;
    }
    public boolean isAtTarget() {
        return Math.abs(liftMotor.getCurrentPosition() - targetPositionCount) < 20;
    }
}
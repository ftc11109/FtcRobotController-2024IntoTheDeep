package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Suspension {

    protected static DcMotor suspensionMotor;
    private final Telemetry telemetry;
    private final Gamepad gamepad;


    private final ElapsedTime runtime = new ElapsedTime();

    static final int LOW_HARDSTOP = 0;

    /*
    static final int HIGH_HARDSTOP = -5115;
    static final int HANG_POSITION = -2978;
     */

    static final int HIGH_POSITION = -7304;
    static final int ENGAGE_POSITION = -6666;
    static final int HANG_POSITION = -5268;

    static final double MAX_SPEED = 1;

    final boolean isAutonomous;

    public int targetPositionCount;

    // todo: add more static encoder count variables as needed, like high, middle and low positions

    // todo: determine what speed to set the motors to & whether up speed is different than down speed

    public Suspension(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad, boolean isAutonomous) {
        // if linear slide doesn't run in auto, isAutonomous will be unnecessary

        suspensionMotor = hardwareMap.get(DcMotor.class,"suspensionMotor"); // port 3

        this.isAutonomous = isAutonomous;
        this.gamepad      = gamepad;
        this.telemetry    = telemetry;

        suspensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        suspensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // todo: figure out which value is best
        suspensionMotor.setDirection(DcMotor.Direction.FORWARD);
        suspensionMotor.setTargetPosition(LOW_HARDSTOP);
        suspensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        suspensionMotor.setPower(0);

        telemetry.addData("Suspension motor position", "%7d", suspensionMotor.getCurrentPosition());

    }

    /**
     * utilizes DcMotor.Runmode.RUN_TO_POSITION to set the motor's target
     * @param ticks the distance the motor must run to, measured in ticks
     */
    public void setPosition(int ticks) {
        targetPositionCount = ticks;
    }

    private void readGamepad(Gamepad gamepad) {
        if(gamepad.dpad_left || gamepad.dpad_right || gamepad.dpad_up || gamepad.dpad_down) {
            suspensionMotor.setPower(MAX_SPEED);
        }
        if(gamepad.dpad_up) targetPositionCount = HIGH_POSITION;
        if(gamepad.dpad_right) targetPositionCount = ENGAGE_POSITION;
        if(gamepad.dpad_down) targetPositionCount = HANG_POSITION;
        if(gamepad.dpad_left) targetPositionCount = LOW_HARDSTOP;
    }

    public void loop() {
        if (!isAutonomous) readGamepad(gamepad);
        /*
        if (targetPositionCount == LOW_HARDSTOP && isAtTarget(50)) {
            suspensionMotor.setPower(0);
        } else {
            suspensionMotor.setPower(MAX_SPEED);
        }
        */
        suspensionMotor.setTargetPosition(targetPositionCount);
        telemetry.addData("Sus encoder position", suspensionMotor.getCurrentPosition());
        telemetry.addData("set position", targetPositionCount);
        telemetry.addData("!!! Suspension motor power !!!", suspensionMotor.getPower());

    }

    public boolean isAtTarget(int tickThreshold) {
        return Math.abs(suspensionMotor.getCurrentPosition() - targetPositionCount) <= tickThreshold;
    }
    public boolean isAtTarget(int target, int tickThreshold) {
        return (Math.abs(suspensionMotor.getCurrentPosition() - targetPositionCount) <= tickThreshold)
                && targetPositionCount == target;
    }
}



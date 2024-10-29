package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LinearSlideTemplate {

    protected static DcMotor slideMotor;
    private final Telemetry telemetry;
    private final Gamepad gamepad;


    private final ElapsedTime runtime = new ElapsedTime();

    static final int LOW_HARDSTOP = 0;
    static final int HIGH_HARDSTOP = 1000; // placeholder

    final boolean isAutonomous;

    // todo: add more static encoder count variables as needed, like high, middle and low positions

    // todo: determine what speed to set the motors to & whether up speed is different than down speed

    private int motorTickTarget = 0; //this variable is for telemetry

    public LinearSlideTemplate(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad, boolean isAutonomous) {
        // if linear slide doesn't run in auto, isAutonomous will be unnecessary

        slideMotor = hardwareMap.get(DcMotor.class,"slide"); // placeholder value

        this.isAutonomous = isAutonomous;
        this.gamepad = gamepad;
        this.telemetry = telemetry;

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // todo: figure out which value is best
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setTargetPosition(LOW_HARDSTOP);

        telemetry.addData("Slide motor position", "%7d", slideMotor.getCurrentPosition());

    }

    /**
     * utilizes DcMotor.Runmode.RUN_TO_POSITION to set the motor's target
     * @param ticks the distance the motor must run to, measured in ticks
     */
    public void setPosition(int ticks) {
        slideMotor.setTargetPosition(ticks);
        motorTickTarget = ticks;
    }

    private void readGamepad(Gamepad gamepad) {
        /* this is where your controls go. an example of this might look like:

        if (gamepad.a) setPosition(LOW_HARDSTOP)
        if (gamepad.b) setPosition(HIGH_HARDSTOP)

         */
        if (gamepad.a) setPosition(LOW_HARDSTOP);
    }

    public void loop() {
        if (!isAutonomous) readGamepad(gamepad);
    }

    /*
    public void setPosition(int encoderTickAmount) {
        targetPositionCount = encoderTickAmount;
        currentSetPosition  = encoderTickAmount;
    }
     */


}

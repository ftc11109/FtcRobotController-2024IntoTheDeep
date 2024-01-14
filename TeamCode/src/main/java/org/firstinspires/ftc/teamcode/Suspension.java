package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Suspension {

    protected static DcMotor susMotor;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final Gamepad gamepad;
    private final ElapsedTime runtime = new ElapsedTime();

    private int targetPosition = EncoderCount.LOW;

    public int currentSetPosition = 0;

    /**
     * Houses LOW and HIGH encoder tick amounts for suspension motor.
     */
    public static class EncoderCount {
        static final int LOW = 0;
        static final int HIGH = -8400; //-8752
    }

    public Suspension(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad = gamepad;

        susMotor = hardwareMap.get(DcMotor.class, "suspension");
        susMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        susMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //you need to set a position count BEFORE switching to runToPosition mode!!
        susMotor.setTargetPosition(EncoderCount.LOW);

        telemetry.addData("Suspension Motor Starting At",  "%7d",
                susMotor.getCurrentPosition());
        susMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void SetPosition(int position) {

            if (position == 0) {
                targetPosition = EncoderCount.LOW;
                currentSetPosition = 0;
            } else if (position == 1) {
                targetPosition = EncoderCount.HIGH;
                currentSetPosition = 1;
            }
        susMotor.setTargetPosition(targetPosition);
    }

    public void loop() {
            //if (gamepad.dpad_down) susMotor.setPower(-0.1);
            //else if (gamepad.dpad_up) susMotor.setPower(0.1);
            //else susMotor.setPower(0);
        //susMotor.setPower(gamepad.left_stick_y * 0.5);

        //targetPosition = Range.clip((int)(targetPosition + gamepad.left_stick_y * 0.5), EncoderCount.HIGH, EncoderCount.LOW);
        //susMotor.setTargetPosition(targetPosition);

        if((gamepad.left_stick_y < -0.2) && (susMotor.getCurrentPosition() > EncoderCount.HIGH)) susMotor.setPower(-0.75);
        else if((gamepad.left_stick_y > 0.2) && (susMotor.getCurrentPosition() < EncoderCount.LOW)) susMotor.setPower(0.75);
        else susMotor.setPower(0);

        telemetry.addData("Left stick Y", gamepad.left_stick_y);
        telemetry.addData("Suspension Motor encoder count is", susMotor.getCurrentPosition());


    }

    public void initLoop() {
        //telemetry.addData("Swing Arm Motor 1 Position is:", swingArmMotor.getCurrentPosition());
        telemetry.addData("Suspension Motor encoder count is", susMotor.getCurrentPosition());
    }
}

/* SUSpension pseudocode

port: 2, name: "suspension"

1: import variables/functions for 5:1 gear ratio and encoder ticks per revolution
2: set motor idle behavior to brake
3:

 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeWrist {

    protected static DcMotor wristMotor;
    private final Gamepad gamepad; //if the wrist is automated, gamepad input might be unnecessary
    private final ElapsedTime runtime = new ElapsedTime();
    private final Telemetry telemetry;

    // todo: figure out values for all placeholders
    static final int ZERO_HARDSTOP = 5;
    static final int DEPLOYED_POSITION = -1070; // placeholder
    //static final int LIFTED_POSITION = 0;
    static final int TRANSFER_POSITION = 5; // placeholder
    // add statics as necessary

    private int motorTickTarget = 0; // this variable is for telemetry

    final boolean isAutonomous; // will not be necessary if wrist is automated

    public IntakeWrist(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad, boolean isAutonomous) {

        wristMotor = hardwareMap.get(DcMotor.class, "wristMotor"); // port 1

        this.gamepad = gamepad;
        this.isAutonomous = isAutonomous;
        this.telemetry = telemetry;

        wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wristMotor.setDirection(DcMotor.Direction.FORWARD);
        wristMotor.setTargetPosition(ZERO_HARDSTOP);
        wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wristMotor.setPower(1);


        telemetry.addData("Slide motor position", "%7d", wristMotor.getCurrentPosition());
    }

    /**
     * utilizes DcMotor.Runmode.RUN_TO_POSITION to set the motor's target
     * @param ticks the distance the motor must run to, measured in ticks
     */
    public void setPosition(int ticks) {
        wristMotor.setTargetPosition(ticks);
        motorTickTarget = ticks;
    }

    private void readGamepad(Gamepad gamepad) { // will be unnecessary if wrist is automated
        if (gamepad.dpad_up  ) setPosition(TRANSFER_POSITION);
        if (gamepad.dpad_down) setPosition(DEPLOYED_POSITION);
    }

    public void loop() {
        if (!isAutonomous) readGamepad(gamepad);
        telemetry.addData("Wrist encoder position", wristMotor.getCurrentPosition());
    }


}
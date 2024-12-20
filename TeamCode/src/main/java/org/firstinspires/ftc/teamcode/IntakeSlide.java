package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSlide {

    protected static DcMotor intakeSlideMotor;
    private final Gamepad gamepad;
    private final Telemetry telemetry;
    private final ElapsedTime runtime = new ElapsedTime();

    static final int LOW_HARDSTOP = 2;
    static final int HIGH_HARDSTOP = GoBildaInchesToTicks.InchesToTicks(16.5, GoBildaInchesToTicks.GoBilda_223rpm); //2880
    static double maxSpeed = 1;
    static double adjustmentModifier = 50;

    // 120mm per rotation

    // todo: make future statics public so they can be used externally, in setPosition()

    final boolean isAutonomous;

    public int targetPositionCount;
    public int motorTickTarget = 0; //this variable is for telemetry

    public IntakeSlide(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad, boolean isAutonomous) {

        intakeSlideMotor = hardwareMap.get(DcMotor.class,"intakeSlideMotor"); // port 0

        this.isAutonomous = isAutonomous;
        this.gamepad = gamepad;
        this.telemetry = telemetry;

        if(isAutonomous) maxSpeed = 0.75;
        else maxSpeed = 1;

        intakeSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // todo: figure out which value is best
        intakeSlideMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeSlideMotor.setTargetPosition(LOW_HARDSTOP);
        intakeSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeSlideMotor.setPower(maxSpeed);

        telemetry.addData("Intake slide motor position", "%7d", intakeSlideMotor.getCurrentPosition());

    }

    /**
     * utilizes DcMotor.Runmode.RUN_TO_POSITION to set the motor's target
     * @param ticks the distance the motor must run to, measured in ticks
     */
    public void setPosition(int ticks) {
        targetPositionCount = ticks;
        motorTickTarget     = ticks;
    }

    private void readGamepad(Gamepad gamepad) {

        if (gamepad.right_stick_y > 0.1 || gamepad.right_stick_y < -0.1 ) {

            targetPositionCount = Range.clip((int)(targetPositionCount + adjustmentModifier *-gamepad.right_stick_y), LOW_HARDSTOP, HIGH_HARDSTOP);

            telemetry.addData("Manual Branch", "Adjustment made");

        } else if (!intakeSlideMotor.isBusy()) {

            //This is so that if you let go of the joystick, it immediately stops the arm from moving. Not a bug!!!

            targetPositionCount = Range.clip(intakeSlideMotor.getCurrentPosition(), LOW_HARDSTOP, HIGH_HARDSTOP);
            intakeSlideMotor.setTargetPosition(targetPositionCount);

            telemetry.addData("Manual Branch", "Stop moving");
        } else {
            telemetry.addData("Manual Branch", "Running to Junction");

        }
    }

    public void loop() {
        if (!isAutonomous) readGamepad(gamepad);
        intakeSlideMotor.setTargetPosition(targetPositionCount);
        telemetry.addData("Slide encoder position", intakeSlideMotor.getCurrentPosition());
        telemetry.addData("The math", HIGH_HARDSTOP);
    }

    public boolean isAtTarget(int tickThreshold) {
        return Math.abs(intakeSlideMotor.getCurrentPosition() - targetPositionCount) <= tickThreshold;
    }

    /*
    public void setPosition(int encoderTickAmount) {
        targetPositionCount = encoderTickAmount;
        currentSetPosition  = encoderTickAmount;
    }
     */


}

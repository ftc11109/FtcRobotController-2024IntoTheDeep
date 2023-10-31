package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake {

   DcMotor intakeMotor;
   private static final double MAX_SPEED = 0.5;

   void pickUpPixel() {
       intakeMotor.setPower(MAX_SPEED);
       MiscFunc.wait(1000); //1000 is a placeholder value
       intakeMotor.setPower(0);
   }

   void ejectPixel() {
       intakeMotor.setPower(-MAX_SPEED);
       MiscFunc.wait(1000); //1000 is a placeholder value
       intakeMotor.setPower(0);
   }

}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
public class Intake {
   DcMotor intakeMotor;
   private static final double MAX_SPEED = 0.5;

   void pickUpPixel(){
       //This line will pick up the pixel
       intakeMotor.setPower(MAX_SPEED);


   }

   void ejectPixel() {
       intakeMotor.setPower(-MAX_SPEED);
       //add wait x seconds
   }

}

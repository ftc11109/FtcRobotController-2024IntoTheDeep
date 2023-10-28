package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake {

    public static void wait(int millis)
    {
        try
        {
            Thread.sleep(millis);
        }
        catch(InterruptedException ex)
        {
            Thread.currentThread().interrupt();
        }
    }

   DcMotor intakeMotor;
   private static final double MAX_SPEED = 0.5;

   void pickUpPixel() {
       intakeMotor.setPower(MAX_SPEED);
       wait(1000); //1000 is a placeholder value
       intakeMotor.setPower(0);
   }

   void ejectPixel() {
       intakeMotor.setPower(-MAX_SPEED);
       wait(1000); //1000 is a placeholder value
       intakeMotor.setPower(0);
   }

}

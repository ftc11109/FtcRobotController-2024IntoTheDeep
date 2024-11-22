package org.firstinspires.ftc.teamcode;

public class GoBildaInchesToTicks {

    //static double GoBilda_60rpm = ((((1+(46.0/17.0))) * (1+(46.0/11.0))) * (1+(46.0/11.0)) * 28) ;

    static double GoBilda_223rpm = ((((1+(46.0/11.0))) * (1+(46.0/11.0))) * 28);

    static double GoBilda_435rpm = ((((1+(46.0/17.0))) * (1+(46.0/17.0))) * 28);

    static double GoBilda_84rpm = ((((1+(46.0/17.0))) * (1+(46.0/17.0))) * (1+(46.0/11.0)) * 28);

    static double VIPER_SLIDE_ONE_ROTATION_INCHES = 120 /*mm*/ / 25.4 ;
    public static int InchesToTicks(double distanceInches, double motorType) {
        // 1 full rotation: 120mm or 4.72441 in
        // distanceInches / 25.4
        // divide wanted distance by full rotation distance then multiply by encoder ticks
        // return tickAmount
        double fullRotations = (distanceInches / VIPER_SLIDE_ONE_ROTATION_INCHES);
        return (int) (fullRotations * motorType);

    }
}


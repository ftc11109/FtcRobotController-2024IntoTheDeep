package org.firstinspires.ftc.teamcode;

public class MiscFunc {
    public static int boolToInt(boolean foo) {
        return (foo) ? 1 : 0;
    }

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
}

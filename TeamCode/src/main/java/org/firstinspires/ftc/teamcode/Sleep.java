package org.firstinspires.ftc.teamcode;

public class Sleep {

    /**
     * Sleeps.
     */
    public static void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public static void STFU() {}
}

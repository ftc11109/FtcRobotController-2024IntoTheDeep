package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DroneLauncherServo {
    ServoController doorServo = new ServoController();
    private HardwareMap hardwareMap;
    private Gamepad gamepad;


    public DroneLauncherServo(HardwareMap hardwareMap, Gamepad gamepad) {
        this.gamepad = gamepad;
        doorServo.init(hardwareMap, "droneLauncherServo");
    }

    public void loop() {
        if (gamepad.dpad_up) {
            doorServo.setServoPosition(0.66);
        } else if (gamepad.dpad_down) {
            doorServo.setServoPosition(0.60);
        } else if (gamepad.dpad_left || gamepad.dpad_right) {
            doorServo.setServoPosition(0.50);
        }
    }
}
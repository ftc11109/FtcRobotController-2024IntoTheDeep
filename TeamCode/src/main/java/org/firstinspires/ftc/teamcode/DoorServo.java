package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;

public class DoorServo {
    ServoController doorServo = new ServoController();
    private HardwareMap hardwareMap;
    private Gamepad gamepad;


    public DoorServo(HardwareMap hardwareMap, Gamepad gamepad) {
        this.gamepad = gamepad;
        doorServo.init(hardwareMap, "doorServo");
    }

    public void loop() {
        if (SwingArm.armMotor.getCurrentPosition() < 25) {
            SetState(2);
        } else if (SwingArm.armMotor.getCurrentPosition() < 800) {
            SetState(0);
        } else {
            if(gamepad.right_trigger > 0) {
                SetState(2);
            } else if (gamepad.left_trigger > 0) {
                SetState(1);
            } else {
                SetState(0);
            }
        }
    }

    /** This is fine.
     * @param state 0: Door is closed
     *              1: Door is halfway open
     *              2: Door is fully open
     */
    public void SetState(int state) {
        if(state == 0) {
            doorServo.setServoPosition(0);
        } else if (state == 1) {
            doorServo.setServoPosition(0.43);
        } else if (state == 2) {
            doorServo.setServoPosition(1);
        }
    }
}
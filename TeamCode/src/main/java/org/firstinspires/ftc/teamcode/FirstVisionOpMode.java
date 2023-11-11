package org.firstinspires.ftc.teamcode;
//package org.firstinspires.ftc.teamcode.opmodes;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.processors.FirstVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
@Autonomous()
public class FirstVisionOpMode extends OpMode {

    private FirstVisionProcessor visionProcessor;

    private VisionPortal visionPortal;

    @Override
    public void init() {
        visionProcessor = new FirstVisionProcessor();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), visionProcessor);
        visionProcessor.getSelection();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        visionPortal.stopStreaming();
    }

    @Override
    public void loop() {
        telemetry.addData("Identified", visionProcessor.getSelection());
        telemetry.addLine("");
        telemetry.addData("Left selection saturation", FirstVisionProcessor.getAvgSaturation(FirstVisionProcessor.hsvMat, FirstVisionProcessor.rectLeft));
        telemetry.addData("Middle selection saturation", FirstVisionProcessor.getAvgSaturation(FirstVisionProcessor.hsvMat, FirstVisionProcessor.rectMiddle));
        telemetry.addData("Right selection saturation", FirstVisionProcessor.getAvgSaturation(FirstVisionProcessor.hsvMat, FirstVisionProcessor.rectRight));

        /* Avg Saturation Results (parentheses indicate control/default value)
        Blue:
            Left: 217 (15)
            Middle: 191 (10)
            Right: 132 (36)
        Red:
            Left:
            Middle:
            Right:

            Threshold of around 75 should work, below this value vision processor should output NONE
         */
    }
}

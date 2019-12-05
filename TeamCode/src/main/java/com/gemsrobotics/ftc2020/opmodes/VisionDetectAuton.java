package com.gemsrobotics.ftc2020.opmodes;

import com.gemsrobotics.ftc2020.vision.RecognitionComparator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class VisionDetectAuton extends BaseAutonomousMode {
    @Override
    public void runAutonomous() {
        waitForStart();

        while (opModeIsActive()) {
            stoneLocator.update();
            telemetry.addData("Stone Location", stoneLocator.getObservedSkyStoneLocation(RecognitionComparator.RIGHTMOST));
            telemetry.update();
        }
    }
}

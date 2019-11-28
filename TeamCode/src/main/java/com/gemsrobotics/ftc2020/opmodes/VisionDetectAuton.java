package com.gemsrobotics.ftc2020.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class VisionDetectAuton extends BaseAutonomousMode {
    @Override
    public void runAutonomous() {
        waitForStart();

        while (!isStopRequested()) {
            stoneLocator.update();
            telemetry.update();
        }
    }
}

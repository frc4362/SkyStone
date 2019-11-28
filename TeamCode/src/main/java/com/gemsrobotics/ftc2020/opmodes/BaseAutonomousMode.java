package com.gemsrobotics.ftc2020.opmodes;

import com.gemsrobotics.ftc2020.vision.SkyStoneLocator;
import com.gemsrobotics.ftc2020.Superstructure;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class BaseAutonomousMode extends LinearOpMode {
    protected Superstructure superstructure;
    protected SkyStoneLocator stoneLocator;

    @Override
    public void runOpMode() {
        superstructure = new Superstructure(hardwareMap, telemetry);
        stoneLocator = new SkyStoneLocator(hardwareMap, telemetry);

        stoneLocator.start();

        runAutonomous();

        stoneLocator.stop();
    }

    public abstract void runAutonomous();
}

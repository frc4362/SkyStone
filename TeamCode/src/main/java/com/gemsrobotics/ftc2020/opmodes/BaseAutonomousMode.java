package com.gemsrobotics.ftc2020.opmodes;

import com.gemsrobotics.ftc2020.hardware.Draggers;
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

        superstructure.draggers.setGoal(Draggers.Goal.RETRACTED);
        superstructure.passthrough.setPosition(Superstructure.PASSTHROUGH_FORWARD_POSITION);
        superstructure.flipper.setPosition(Superstructure.FLIPPER_UPRIGHT_POSITION);

        boolean crashed = false;
        String crashMessage = null;

        try {
            stoneLocator.start();
            runAutonomous();
        } catch (final Throwable throwable) {
            crashed = true;
            crashMessage = throwable.getMessage();
        } finally {
            telemetry.addLine(crashed ? "Auton crashed." : "Auton complete.");

            if (crashMessage != null) {
                telemetry.addLine(crashMessage);
            }

            telemetry.update();
            stoneLocator.stop();
        }
    }

    protected void fulfillRequests() {
        if (opModeIsActive()) {
            do {
                superstructure.update();
            } while (opModeIsActive() && superstructure.isBusy());
        }
    }

    public abstract void runAutonomous();
}

package com.gemsrobotics.ftc2020.opmodes;

import com.gemsrobotics.ftc2020.Constants;
import com.gemsrobotics.ftc2020.hardware.Draggers;
import com.gemsrobotics.ftc2020.vision.SkyStoneLocator;
import com.gemsrobotics.ftc2020.Superstructure;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static java.lang.StrictMath.signum;
import static java.lang.StrictMath.toDegrees;
import static java.lang.StrictMath.toRadians;

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

    private final double kP = 0.01;
    protected final void driveDistance(double speed, double distance) {
        final RigidTransform startPose = superstructure.chassis.getRigidTransformEstimate();
        RigidTransform currentPose;

       do {
           currentPose = superstructure.chassis.getRigidTransformEstimate();
           superstructure.chassis.setOpenLoopCurvature(
                   speed,
                   kP * startPose.getRotation().difference(currentPose.getRotation()).getDegrees(),
                   false);
           superstructure.chassis.update();
           telemetry.addData("Pose Estimate", currentPose);
           telemetry.update();
       } while (opModeIsActive() && currentPose.relativeTo(startPose).getTranslation().norm() < distance);

       superstructure.chassis.setDisabled();
       superstructure.chassis.update();
    }

    protected final void turn(final Rotation rotation) {
        final Rotation endHeading = superstructure.chassis.getRigidTransformEstimate().getRotation().sum(rotation);

        double headingError;

        do {
            final Rotation currentHeading = superstructure.chassis.getRigidTransformEstimate().getRotation();
            headingError = endHeading.distance(currentHeading);
            superstructure.chassis.setOpenLoopCurvature(0.0, 0.25 * signum(headingError), true);
            superstructure.chassis.update();
        } while (opModeIsActive() && headingError > toRadians(2.0));

        superstructure.chassis.setDisabled();
        superstructure.chassis.update();
    }

    public abstract void runAutonomous();
}

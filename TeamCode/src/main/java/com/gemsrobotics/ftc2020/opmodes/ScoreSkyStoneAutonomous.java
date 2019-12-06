package com.gemsrobotics.ftc2020.opmodes;

import com.gemsrobotics.ftc2020.vision.RecognitionComparator;
import com.gemsrobotics.ftc2020.vision.SkyStoneLocator;
import com.gemsrobotics.lib.Side;
import com.gemsrobotics.lib.math.se2.Rotation;

public abstract class ScoreSkyStoneAutonomous extends BaseAutonomousMode {
    private final Side m_side;

    protected ScoreSkyStoneAutonomous(final Side side) {
        m_side = side;
    }

    @Override
    public void runAutonomous() {
        final RecognitionComparator selectionStrategy = m_side == Side.RIGHT
                ? RecognitionComparator.RIGHTMOST
                : RecognitionComparator.LEFTMOST;

        SkyStoneLocator.Location observedLocation = SkyStoneLocator.Location.UNKNOWN;

        while (!isStarted()) {
            stoneLocator.update();
            observedLocation = stoneLocator.getObservedSkyStoneLocation(selectionStrategy);
            telemetry.addData("Observed SkyStone Location", observedLocation);
            telemetry.update();
        }

        waitForStart();

        driveDistance(0.275, 9.0);

        sleep(200);

        final Rotation angleToBlock;

        if (observedLocation == SkyStoneLocator.Location.WALL) {
            angleToBlock = Rotation.degrees(5 * m_side.turnMultiplier);
        } else if (observedLocation == SkyStoneLocator.Location.BRIDGE) {
            angleToBlock = Rotation.degrees(-27 * m_side.turnMultiplier);
        } else { // CENTER
            angleToBlock = Rotation.degrees(-12 * m_side.turnMultiplier);
        }

        turn(angleToBlock);
//        fulfillRequests();
//
//        superstructure.chassis.setDisabled();
//        fulfillRequests();
//
//        sleep(100);
//        superstructure.requestTurn(turnToBlock);
//        fulfillRequests();
//
//        superstructure.chassis.setDisabled();
//        superstructure.update();

//        sleep(100);
//
//        final RigidTransform prePickupPose = superstructure.chassis.getRigidTransformEstimate();
//
//        superstructure.setGoal(Superstructure.Goal.INTAKING);
//        fulfillRequests();
//
//        driveDistance(0.5, 45.0);
//
//        sleep(1000);
//
//        superstructure.chassis.setDisabled();
//        superstructure.requestGrab();
//
//        fulfillRequests();
//
//        sleep(100);
//        superstructure.requestDrive(-50);
//
//        fulfillRequests();
//
//        superstructure.chassis.setDisabled();
//
//        fulfillRequests();
//
//        sleep(100);
//        superstructure.requestTurn(Rotation.degrees(90 * m_side.turnMultiplier).difference(turnToBlock.inverse()));
//
//        fulfillRequests();
//
//        superstructure.chassis.setDisabled();
//
//        fulfillRequests();
//
//        sleep(100);
//        superstructure.requestDrive(-70);
//
//        fulfillRequests();
//
//        superstructure.chassis.setDisabled();
//
//        fulfillRequests();

        telemetry.addLine("Done!");
        telemetry.update();
    }
}

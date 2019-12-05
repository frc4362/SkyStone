package com.gemsrobotics.ftc2020.opmodes;

import com.gemsrobotics.ftc2020.Superstructure;
import com.gemsrobotics.ftc2020.vision.RecognitionComparator;
import com.gemsrobotics.ftc2020.vision.SkyStoneLocator;
import com.gemsrobotics.lib.Side;
import com.gemsrobotics.lib.math.se2.RigidTransform;
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

        superstructure.requestDrive(13.0);

        fulfillRequests();

        superstructure.chassis.setDisabled();
        superstructure.update();

        sleep(200);

        final Rotation turnToBlock;

        if (observedLocation == SkyStoneLocator.Location.WALL) {
            turnToBlock = Rotation.degrees(5 * m_side.turnMultiplier);
        } else if (observedLocation == SkyStoneLocator.Location.BRIDGE) {
            turnToBlock = Rotation.degrees(-27 * m_side.turnMultiplier);
        } else { // CENTER
            turnToBlock = Rotation.degrees(-12 * m_side.turnMultiplier);
        }

        superstructure.requestTurn(turnToBlock);

        fulfillRequests();

        superstructure.chassis.setDisabled();
        superstructure.update();

        sleep(100);

        final RigidTransform currentPose = superstructure.chassis.getRigidTransformEstimate();

        superstructure.setGoal(Superstructure.Goal.INTAKING);
        superstructure.requestDrive(45);

        fulfillRequests();

        superstructure.chassis.setDisabled();
        superstructure.requestGrab();

        fulfillRequests();

        superstructure.setGoal(Superstructure.Goal.STOWED);
        superstructure.requestDrive(-45);

        fulfillRequests();

        superstructure.chassis.setDisabled();

        fulfillRequests();

        sleep(100);
        superstructure.requestTurn(Rotation.degrees(-90 * m_side.turnMultiplier).sum(turnToBlock.inverse()));

        fulfillRequests();

        superstructure.chassis.setDisabled();

        fulfillRequests();

        sleep(100);
        superstructure.requestDrive(70);

        fulfillRequests();

        superstructure.chassis.setDisabled();

        fulfillRequests();

        telemetry.addLine("Done!");
        telemetry.update();
    }
}

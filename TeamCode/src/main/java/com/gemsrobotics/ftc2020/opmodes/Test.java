package com.gemsrobotics.ftc2020.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.gemsrobotics.ftc2020.hardware.MecanumChassis;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static com.gemsrobotics.lib.utils.MathUtils.Tau;

@Autonomous
public class Test extends LinearOpMode {
	@Override
	public void runOpMode() {
		MecanumChassis drive = new MecanumChassis(hardwareMap);

		Trajectory trajectory = drive.getTrajectoryBuilder()
				.splineTo(new Pose2d(30, -30, Tau / 4))
				.build();

		waitForStart();

		if (isStopRequested()) return;

		drive.setTrajectoryGoal(trajectory);

		while (!isStopRequested() && drive.isBusy()) {
			drive.update();
			telemetry.addData("Pose Estimate", drive.getPoseEstimate());
		}

		drive.setDisabled();
	}
}

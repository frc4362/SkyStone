package com.gemsrobotics.ftc2020.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.gemsrobotics.ftc2020.hardware.Chassis;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static com.gemsrobotics.lib.utils.MathUtils.Tau;

@Autonomous
public class Test extends LinearOpMode {
	@Override
	public void runOpMode() {
		final Chassis drive = new Chassis(hardwareMap);

		Trajectory trajectory = drive.getTrajectoryBuilder()
				.splineTo(new Pose2d(70 * 1.6, -30 * 1.28, 0))
//				.splineTo(new Pose2d(80, -30, 0))
//				.splineTo(new Pose2d(70, 0, Tau / 2))
//				.forward(55)
				.build();

		waitForStart();

		if (isStopRequested()) {
			return;
		}

		drive.setTrajectoryGoal(trajectory);

		while (!isStopRequested() && drive.isBusy()) {
			drive.update();
			telemetry.addData("Pose Estimate", drive.getPoseEstimate());
			telemetry.update();
		}

		drive.setDisabled();

		while (!isStopRequested()) {
			telemetry.addData("Pose Estimate", drive.getPoseEstimate());
			telemetry.update();
		}
	}
}

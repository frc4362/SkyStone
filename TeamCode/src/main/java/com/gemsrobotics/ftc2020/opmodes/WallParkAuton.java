package com.gemsrobotics.ftc2020.opmodes;

import com.gemsrobotics.ftc2020.Superstructure;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Wall Park Auton")
public class WallParkAuton extends LinearOpMode {
	private Superstructure superstructure;

	@Override
	public void runOpMode() {
//		Trajectory trajectory = superstructure.chassis.getTrajectoryBuilder()
//				.reverse()
//				.forward(15.0)
//				.build();

		superstructure = new Superstructure(hardwareMap, telemetry);

		waitForStart();

		final long endtime = System.currentTimeMillis() + 750;
		superstructure.chassis.setOpenLoopCurvature(0.6, 0.0, false);

//		superstructure.chassis.setTrajectoryGoal(trajectory);

		while (!isStopRequested() && endtime > System.currentTimeMillis()) {
			superstructure.chassis.update();
		}

		superstructure.chassis.applySafeState();
	}
}

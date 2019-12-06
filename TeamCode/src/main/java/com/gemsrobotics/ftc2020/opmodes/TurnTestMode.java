package com.gemsrobotics.ftc2020.opmodes;

import com.gemsrobotics.ftc2020.hardware.Chassis;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static com.gemsrobotics.lib.utils.MathUtils.Tau;

@Autonomous
public class TurnTestMode extends LinearOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		final Chassis chassis = new Chassis(hardwareMap);

		waitForStart();

		chassis.setTurnGoal(Tau / 4);

		while (!isStopRequested()) {
			chassis.update();
		}
	}
}

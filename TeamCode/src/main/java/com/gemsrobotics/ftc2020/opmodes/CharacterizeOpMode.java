package com.gemsrobotics.ftc2020.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.tuning.AccelRegression;
import com.acmerobotics.roadrunner.tuning.RampRegression;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.gemsrobotics.ftc2020.hardware.Chassis;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.system.Misc;

import static com.gemsrobotics.ftc2020.Constants.getMaxDriveRpm;
import static com.gemsrobotics.ftc2020.Constants.driveRpmToVelocity;

@Autonomous
public class CharacterizeOpMode extends LinearOpMode {
	public static final double MAX_POWER = 0.3;
	public static final double DISTANCE = 100;

	@Override
	public void runOpMode() throws InterruptedException {
		Chassis drive = new Chassis(hardwareMap);

		NanoClock clock = NanoClock.system();

		telemetry.log().add("Press play to begin the feedforward tuning routine");
		telemetry.update();

		waitForStart();

		if (isStopRequested()) return;

		telemetry.log().clear();
		telemetry.log().add("Would you like to fit kStatic?");
		telemetry.log().add("Press (A) for yes, (B) for no");
		telemetry.update();

		boolean fitIntercept = false;
		while (!isStopRequested()) {
			if (gamepad1.a) {
				fitIntercept = true;
				while (!isStopRequested() && gamepad1.a) {
					idle();
				}
				break;
			} else if (gamepad1.b) {
				while (!isStopRequested() && gamepad1.b) {
					idle();
				}
				break;
			}
			idle();
		}

		telemetry.log().clear();
		telemetry.log().add(Misc.formatInvariant(
				"Place your robot on the field with at least %.2f in of room in front", DISTANCE));
		telemetry.log().add("Press (A) to begin");
		telemetry.update();

		while (!isStopRequested() && !gamepad1.a) {
			idle();
		}
		while (!isStopRequested() && gamepad1.a) {
			idle();
		}

		telemetry.log().clear();
		telemetry.log().add("Running...");
		telemetry.update();

		double maxVel = driveRpmToVelocity(getMaxDriveRpm());
		double finalVel = MAX_POWER * maxVel;
		double accel = (finalVel * finalVel) / (2.0 * DISTANCE);
		double rampTime = Math.sqrt(2.0 * DISTANCE / accel);

		double startTime = clock.seconds();
		RampRegression rampRegression = new RampRegression();

		drive.setPoseEstimate(new Pose2d());
		while (!isStopRequested()) {
			double elapsedTime = clock.seconds() - startTime;
			if (elapsedTime > rampTime) {
				break;
			}

			double vel = accel * elapsedTime;
			double power = vel / maxVel;

			rampRegression.add(elapsedTime, drive.getPoseEstimate().getX(), power);

			drive.setDrivePower(new Pose2d(power, 0.0, 0.0));
			drive.updatePoseEstimate();
		}

		drive.setDrivePower(new Pose2d(0.0, 0.0, 0.0));

		RampRegression.RampResult rampResult = rampRegression.fit(fitIntercept);

		telemetry.log().clear();
		telemetry.log().add("Quasi-static ramp up test complete");
		if (fitIntercept) {
			telemetry.log().add(Misc.formatInvariant("kV = %.5f, kStatic = %.5f (R^2 = %.2f)",
					rampResult.kV, rampResult.kStatic, rampResult.rSquare));
		} else {
			telemetry.log().add(Misc.formatInvariant("kV = %.5f (R^2 = %.2f)",
					rampResult.kStatic, rampResult.rSquare));
		}
		telemetry.log().add("Would you like to fit kA?");
		telemetry.log().add("Press (A) for yes, (B) for no");
		telemetry.update();

		boolean fitAccelFF = false;
		while (!isStopRequested()) {
			if (gamepad1.a) {
				fitAccelFF = true;
				while (!isStopRequested() && gamepad1.a) {
					idle();
				}
				break;
			} else if (gamepad1.b) {
				while (!isStopRequested() && gamepad1.b) {
					idle();
				}
				break;
			}
			idle();
		}

		if (fitAccelFF) {
			telemetry.log().clear();
			telemetry.log().add("Place the robot back in its starting position");
			telemetry.log().add("Press (A) to continue");
			telemetry.update();

			while (!isStopRequested() && !gamepad1.a) {
				idle();
			}
			while (!isStopRequested() && gamepad1.a) {
				idle();
			}

			telemetry.log().clear();
			telemetry.log().add("Running...");
			telemetry.update();

			double maxPowerTime = DISTANCE / maxVel;

			startTime = clock.seconds();
			AccelRegression accelRegression = new AccelRegression();

			drive.setPoseEstimate(new Pose2d());
			drive.setDrivePower(new Pose2d(MAX_POWER, 0.0, 0.0));
			while (!isStopRequested()) {
				double elapsedTime = clock.seconds() - startTime;
				if (elapsedTime > maxPowerTime) {
					break;
				}

				accelRegression.add(elapsedTime, drive.getPoseEstimate().getX(), MAX_POWER);

				drive.updatePoseEstimate();
			}
			drive.setDrivePower(new Pose2d(0.0, 0.0, 0.0));

			AccelRegression.AccelResult accelResult = accelRegression.fit(rampResult.kV, rampResult.kStatic);

			telemetry.log().clear();
			telemetry.log().add("Constant power test complete");
			telemetry.log().add(Misc.formatInvariant("kA = %.5f (R^2 = %.2f)",
					accelResult.kA, accelResult.rSquare));
			telemetry.update();
		}

		while (!isStopRequested()) {
			idle();
		}
	}
}
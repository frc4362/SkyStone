package com.gemsrobotics.ftc2020;

import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import static com.gemsrobotics.lib.utils.MathUtils.Tau;

public class Constants {
	public static final MotorConfigurationType MOTOR_CONFIG =
			MotorConfigurationType.getMotorType(NeveRest20Gearmotor.class);
	public static final double NEVEREST_TICKS_PER_REV = MOTOR_CONFIG.getTicksPerRev();

	public static final double
			WHEEL_RADIUS = 2.0,
			DRIVE_GEAR_RATIO = 1.5,
			TRACK_WIDTH = 29,
			WHEEL_BASE = 13.625;

	public static final double
			kS = 0.07134,
			kV = 0.0117,
			kA = 0.00066;

	public static DriveConstraints TRACKING_CONSTRAINTS = new DriveConstraints(
			30.0,
			40.0,
			0.0,
			Math.toRadians(180.0),
			Math.toRadians(180.0),
			0.0
	);

	// magic numbeeeeers
	public static double localizerEncoderTicksToInches(int ticks) {
		return Tau * 1.0 * (ticks / 2400.0);
	}

	public static double driveEncoderTicksToInches(int ticks) {
		return WHEEL_RADIUS * Tau * DRIVE_GEAR_RATIO * ticks / NEVEREST_TICKS_PER_REV;
	}

	public static double driveRpmToVelocity(double rpm) {
		return rpm * DRIVE_GEAR_RATIO * Tau * WHEEL_RADIUS / 60.0;
	}

	public static double getMaxDriveRpm() {
		return MOTOR_CONFIG.getMaxRPM();
	}
}

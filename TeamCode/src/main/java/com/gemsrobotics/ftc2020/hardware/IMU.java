package com.gemsrobotics.ftc2020.hardware;

import com.gemsrobotics.lib.math.se2.Rotation;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class IMU {
	private final BNO055IMU m_internal;

	public IMU(final BNO055IMU imu) {
		m_internal = imu;

		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
		parameters.mode = BNO055IMU.SensorMode.IMU;
		parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
		parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
		parameters.loggingEnabled = false;

		imu.initialize(parameters);
	}

	public boolean isCalibrated() {
		return m_internal.isGyroCalibrated();
	}

	public Rotation getHeading() {
		final double yaw = m_internal.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
		return Rotation.degrees(yaw);
	}
}

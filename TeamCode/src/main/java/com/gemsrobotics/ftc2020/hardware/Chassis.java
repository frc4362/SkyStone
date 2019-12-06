package com.gemsrobotics.ftc2020.hardware;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.gemsrobotics.ftc2020.Constants;
import com.gemsrobotics.ftc2020.Subsystem;
import com.gemsrobotics.lib.controls.PIDFController;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.math.se2.Translation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static com.gemsrobotics.lib.utils.MathUtils.Tau;
import static com.gemsrobotics.lib.utils.MathUtils.deadband;
import static com.gemsrobotics.lib.utils.MathUtils.lerp;
import static com.gemsrobotics.lib.utils.MathUtils.limit;
import static com.gemsrobotics.lib.utils.MathUtils.normalize;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.signum;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

public final class Chassis extends MecanumDrive implements Subsystem {
	public static final double
		kX = 1.6,
		kY = 1.28;

	private static final double
			FAST_ROTATE_SCALAR = 0.7,
			SLOW_ROTATE_SCALAR = 0.33,
			DIRECTION_SNAP_THRESHOLD_RADIANS = toRadians(10.0),
			INPUT_DEADBAND = 0.2;
	private static final PIDCoefficients
			X_TRANSLATION_CONTROLLER = new PIDCoefficients(1.4, 0.0, 3.0),
			Y_TRANSLATION_CONTROLLER = new PIDCoefficients(5.1, 0.0, 3.0),
			HEADING_CONTROLLER = new PIDCoefficients(6.0, 0.0, 0.0);

	private final DcMotorEx
			m_leftFrontMotor,
			m_leftBackMotor,
			m_rightFrontMotor,
			m_rightBackMotor;
	private final List<DcMotorEx> m_motors;
	private final IMU m_imu;
	private final PIDFController m_openLoopRotationController;
	private final com.acmerobotics.roadrunner.control.PIDFController m_turnController;
	private final Localizer m_localization;
	private final DriveConstraints m_constraints;
	private final TrajectoryFollower m_follower;

	private MotionProfile m_turnProfile;
	private double m_turnStart;

	private ControlMode m_mode;
	private Goal m_goal;
	private double m_accumulator;
	private boolean m_lastDoHeadingLock;

	public Chassis(final HardwareMap hardware) {
		super(
			Constants.kV,
			Constants.kA,
			Constants.kS,
			Constants.TRACK_WIDTH,
			Constants.WHEEL_BASE);

		m_leftFrontMotor = hardware.get(DcMotorEx.class, "FrontLeftMotor");
		m_leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
		m_leftBackMotor = hardware.get(DcMotorEx.class, "BackLeftMotor");
		m_leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
		m_rightBackMotor = hardware.get(DcMotorEx.class, "BackRightMotor");
		m_rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
		m_rightFrontMotor = hardware.get(DcMotorEx.class, "FrontRightMotor");
		m_rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);

		m_motors = Arrays.asList(
				m_leftFrontMotor,
				m_leftBackMotor,
				m_rightBackMotor,
				m_rightFrontMotor);

		for (final DcMotorEx motor : m_motors) {
			motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		}

		m_imu = new IMU(hardware.get(BNO055IMU.class, "imu"));

		m_openLoopRotationController = new PIDFController(0.0, 0.0, 1.0, 0.0);
		m_openLoopRotationController.setContinuous(true);
		m_openLoopRotationController.setTolerance(1.5);
		m_openLoopRotationController.setOutputRange(-0.15, +0.15);
		m_openLoopRotationController.setInputRange(-180.0, +180.0);

		final com.acmerobotics.roadrunner.control.PIDCoefficients HEADING_PID =
				new com.acmerobotics.roadrunner.control.PIDCoefficients(0.5, 0.0, 0.0);
		m_turnController = new com.acmerobotics.roadrunner.control.PIDFController(HEADING_PID);
		m_turnController.setInputBounds(0, Tau);

		m_localization = new MyLocalizer();

		m_constraints = new MecanumConstraints(
				Constants.TRACKING_CONSTRAINTS,
				Constants.TRACK_WIDTH);

		m_follower = new HolonomicPIDVAFollower(
				X_TRANSLATION_CONTROLLER,
				Y_TRANSLATION_CONTROLLER,
				HEADING_CONTROLLER);

		setDisabled();
		m_lastDoHeadingLock = false;
		m_accumulator = 0.0;
	}

	private class MyLocalizer extends TwoTrackingWheelLocalizer {
		private MyLocalizer() {
			super(Arrays.asList(
					new Pose2d(-6.0, 2.0 + (9.0 / 16.0), 0),
					new Pose2d(-5.5, -3.25, toRadians(90))));
		}

		@Override
		public List<Double> getWheelPositions() {
			final List<Double> ret = new ArrayList<>();
			ret.add(Constants.localizerEncoderTicksToInches(m_leftFrontMotor.getCurrentPosition()));
			ret.add(Constants.localizerEncoderTicksToInches(-m_rightFrontMotor.getCurrentPosition()));
			return ret;
		}

		@Override
		public double getHeading() {
			return Chassis.this.getExternalHeading();
		}
	}

	@Override
	public Localizer getLocalizer() {
		return m_localization;
	}

	@Override
	public List<Double> getWheelPositions() {
		return Arrays.asList(0.0, 0.0, 0.0, 0.0);
	}

	public RigidTransform getRigidTransformEstimate() {
		return RigidTransform.ofPose2d(getPoseEstimate());
	}

	public TrajectoryBuilder getTrajectoryBuilder() {
		return new TrajectoryBuilder(getPoseEstimate(), m_constraints);
	}

	@Override
	public void setMotorPowers(double v0, double v1, double v2, double v3) {
		m_leftFrontMotor.setPower(v0);
		m_leftBackMotor.setPower(v1);
		m_rightBackMotor.setPower(v2);
		m_rightFrontMotor.setPower(v3);
	}

	@Override
	protected double getRawExternalHeading() {
		return m_imu.getHeading().getRadians();
	}

	private enum ControlMode {
		DISABLED, OPEN_LOOP, TURNING, TRACKING
	}

	private void configureMode(final ControlMode newMode) {
		if (newMode != m_mode) {
			for (final DcMotorEx motor : m_motors) {
				switch (newMode) {
					case DISABLED:
						motor.setMotorDisable();
						break;
					case TURNING:
					case OPEN_LOOP:
					case TRACKING:
						motor.setMotorEnable();
						motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
						break;
				}
			}

			m_mode = newMode;
		}
	}

	// This is the way it is because it describes the goals of the subsystem while still
	// being meaningfully composable from several different inputs
	private class Goal {
		private Translation movement;
		private double rotation;
		private boolean isFieldOriented;

		private Goal(final Translation vec, final double z, final boolean isFieldOriented) {
			this.movement = vec;
			this.rotation = z;
			this.isFieldOriented = isFieldOriented;
		}

		private Goal() {
			this(Translation.identity(), 0.0, false);
		}
	}

	public void setOpenLoopCurvature(final double linearPower, double zRotation, final boolean quickturn) {
		configureMode(ControlMode.OPEN_LOOP);

		double overPower, angularPower;

		if (quickturn) {
			if (abs(linearPower) < 0.2) {
				m_accumulator = (0.9) * m_accumulator + 0.1 * limit(zRotation, 1.0) * 2;
			}

			overPower = 1.0;
			angularPower = -zRotation;
		} else {
			overPower = 0.0;
			zRotation *= -signum(linearPower);
			angularPower = abs(linearPower) * zRotation * 1.25 - m_accumulator;

			if (m_accumulator > 1) {
				m_accumulator -= 1;
			} else if (m_accumulator < -1) {
				m_accumulator += 1;
			} else {
				m_accumulator = 0.0;
			}
		}

		double leftPower  = linearPower - angularPower,
			   rightPower = linearPower + angularPower;

		// ensure the appropriate minimum speed based on turn speed
		if (leftPower > 1.0) {
			rightPower -= overPower * (leftPower - 1.0);
			leftPower = 1.0;
		} else if (rightPower > 1.0) {
			leftPower -= overPower * (rightPower - 1.0);
			rightPower = 1.0;
		} else if (leftPower < -1.0) {
			rightPower += overPower * (-1.0 - leftPower);
			leftPower = -1.0;
		} else if (rightPower < -1.0) {
			leftPower += overPower * (-1.0 - rightPower);
			rightPower = -1.0;
		}

		// convert to a holonomic signal
		final Translation linearVector = new Translation((leftPower + rightPower) / 2, 0.0);
		final double zPower = (rightPower - leftPower) / 2;

		m_goal = new Goal(linearVector, zPower, false);
	}

	public void setOpenLoopPolar(
			final double x,
			final double y,
			double z,
			final boolean isFieldOriented,
			final boolean isSlowed
	) {
		configureMode(ControlMode.OPEN_LOOP);

		Translation input = new Translation(x, y);
		double magnitude = input.norm();

		// snap rotation to the nearest pole if it close enough already, and maintain heading while doing so
		// allows for consistent, straight motion in the polar directions
		final Rotation nearestPole = input.direction().getNearestPole(Rotation.degrees(45));
		final boolean doHeadingLock = abs(input.direction().distance(nearestPole)) < DIRECTION_SNAP_THRESHOLD_RADIANS;
		if (doHeadingLock) {
			input = nearestPole.toTranslation().scale(magnitude);

			final double headingDegrees = toDegrees(getExternalHeading());

			if (!m_lastDoHeadingLock) {
				m_openLoopRotationController.setReference(headingDegrees);
			}

			z -= m_openLoopRotationController.update(0.0, headingDegrees);
		} else if (m_lastDoHeadingLock) {
			m_openLoopRotationController.reset();
		}

		// deadband radially
		if (magnitude < INPUT_DEADBAND) {
			input = Translation.identity();
			magnitude = 0.0;
		}

		// scale movement vector to reduce sensitivity on the lower end
		final double scalePower = isSlowed ? 1.75 : 1.5;
		input = Translation.fromPolar(input.direction(), pow(magnitude, scalePower));
		// scale it so you can go full speed at an angle
		final double cardinalError = abs(input.direction().distance(input.direction().getNearestPole(Rotation.degrees(90))));
		input = Translation.fromPolar(input.direction(), lerp(input.norm(), 2 * sqrt(2) * input.norm(), cardinalError / 45.0));
		z = deadband(z, INPUT_DEADBAND);

		if (isSlowed) {
			input = input.scale(SLOW_ROTATE_SCALAR);
			z *= SLOW_ROTATE_SCALAR;
		} else {
			z *= FAST_ROTATE_SCALAR;
		}

		m_goal = new Goal(input, z, isFieldOriented);
		m_lastDoHeadingLock = doHeadingLock;
	}

	public void setTrajectoryGoal(final Trajectory trajectory) {
		configureMode(ControlMode.TRACKING);
		m_follower.followTrajectory(trajectory);
	}

	public void setTurnGoal(final double angle) {
		final double heading = getPoseEstimate().getHeading();
		m_turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
				new MotionState(heading, 0, 0, 0),
				new MotionState(heading + angle, 0, 0, 0),
				m_constraints.maxAngVel,
				m_constraints.maxAngAccel,
				m_constraints.maxAngJerk
		);
		m_turnStart = (System.currentTimeMillis() / 1000.0);
		m_mode = ControlMode.TURNING;
	}

	public void setDisabled() {
		configureMode(ControlMode.DISABLED);
		m_goal = new Goal(Translation.identity(), 0.0, false);
	}

	@Override
	public void applySafeState() {
		setMotorPowers(0.0, 0.0, 0.0, 0.0);
	}

	@Override
	public void update() {
		updatePoseEstimate();

		switch (m_mode) {
			case DISABLED:
				break;
			case OPEN_LOOP:
				updateOpenLoop(m_goal, Rotation.radians(getExternalHeading()));
				break;
			case TURNING:
				double time = (System.currentTimeMillis() / 1000.0) - m_turnStart;

				MotionState targetState = m_turnProfile.get(time);

				m_turnController.setTargetPosition(targetState.getX());

				double targetOmega = targetState.getV();
				double targetAlpha = targetState.getA();
				double correction = m_turnController.update(getPoseEstimate().getHeading(), targetOmega);

				setDriveSignal(new DriveSignal(
						new Pose2d(0, 0, targetOmega + correction),
						new Pose2d(0, 0, targetAlpha)));

				if (time >= m_turnProfile.duration()) {
					configureMode(ControlMode.DISABLED);
					setDriveSignal(new DriveSignal());
				}

				break;
			case TRACKING:
				setDriveSignal(m_follower.update(getPoseEstimate()));

				if (!m_follower.isFollowing()) {
					setDriveSignal(new DriveSignal());
					setDisabled();
					return;
				}

				break;
		}
	}

	private void updateOpenLoop(final Goal goal, final Rotation heading) {
		// Compensate for gyro angle.
		if (goal.isFieldOriented) {
			goal.movement = goal.movement.rotateBy(heading);
		}

		final double magnitude = goal.movement.norm();
		// calculate intended movement direction and compensate for the wheel roller angles
		final double direction = goal.movement.direction().getRadians() + (Tau / 8);

		// calculate the individual duty cycle of each wheel
		// FL, BL, BR, FR
		final double[] wheelSpeeds = new double[] {
				magnitude * sin(direction) + goal.rotation,
				magnitude * cos(direction) + goal.rotation,
				magnitude * sin(direction) - goal.rotation,
				magnitude * cos(direction) - goal.rotation,
		};

		normalize(wheelSpeeds, 1.0);

		setMotorPowers(wheelSpeeds[0], wheelSpeeds[1], wheelSpeeds[2], wheelSpeeds[3]);
	}

	public boolean isBusy() {
		return m_follower.isFollowing() || m_mode == ControlMode.TURNING;
	}
}

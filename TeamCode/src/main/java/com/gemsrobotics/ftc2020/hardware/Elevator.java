package com.gemsrobotics.ftc2020.hardware;

import com.gemsrobotics.ftc2020.Subsystem;
import com.gemsrobotics.lib.DeltaTime;
import com.gemsrobotics.lib.controls.PIDFController;
import com.gemsrobotics.lib.utils.MathUtils;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static com.gemsrobotics.lib.utils.MathUtils.epsilonEquals;

public class Elevator implements Subsystem {
	private static final int HEIGHT = 5900;
	private static final int MOVABLE_HEIGHT = 5750;
	private static final double GRAVITY_FEEDFORWARD = 0.1;
	private static final double TICKS_TO_INCHES = 204.8611;
	private static final double PERCENT_PER_INCH = TICKS_TO_INCHES / HEIGHT;
	private static final double PERCENT_PER_CUBE_INCREMENT = 4.0 * PERCENT_PER_INCH;
	private static final PIDFController.Gains GAINS = new PIDFController.Gains(
			0.004,
			0.0, // 0.004
			0.0,
			0.0
	);

	private static final MathUtils.Bounds SCRUBBING_RANGE = new MathUtils.Bounds(0.1, 0.93);

	protected final DcMotorEx m_motor;
	protected final PIDFController m_controller;
	protected final DeltaTime m_time;

	protected ControlMode m_mode;
	protected double m_goal, m_output;

	public Elevator(final HardwareMap hardwareMap) {
		m_motor = hardwareMap.get(DcMotorEx.class, "LiftVerticalMotor");
		m_motor.setDirection(DcMotorSimple.Direction.FORWARD);
		m_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		m_controller = new PIDFController(GAINS);
		m_controller.setInputRange(0, 5900);
		m_controller.setOutputRange(-0.2, 1.0 - GRAVITY_FEEDFORWARD);
		m_controller.setContinuous(false);
		m_controller.setTolerance(0);
		m_controller.setIntegralRange(0.0);

		m_time = new DeltaTime();

		configureMode(ControlMode.DISABLED);
		m_goal = 0.0;
		m_output = 0.0;
	}

	public enum ControlMode {
		DISABLED, OPEN_LOOP, POSITION
	}

	private void configureMode(final ControlMode newMode) {
		if (newMode != m_mode) {
			switch (newMode) {
				case DISABLED:
					m_motor.setMotorDisable();
					break;
				default:
					m_motor.setMotorEnable();
					break;
			}

			m_mode = newMode;
		}
	}

	public void setOpenLoopGoal(final double power) {
		configureMode(ControlMode.OPEN_LOOP);
		m_goal = power;
	}

	public void setPositionGoal(final double percent) {
		configureMode(ControlMode.POSITION);
		m_goal = percent;
	}

	public void incrementPosition() {
		if (m_mode != ControlMode.POSITION) {
			configureMode(ControlMode.POSITION);
			m_goal = 0.0;
		}

		setPositionGoal(m_goal + PERCENT_PER_CUBE_INCREMENT);
	}

	public void setHoldPositionGoal() {
		setPositionGoal(getCurrentPercent());
	}

	public void setDisabled() {
		configureMode(ControlMode.DISABLED);
		m_goal = 0.0;
	}

	public double getCurrentPosition() {
		return m_motor.getCurrentPosition();
	}

	public double getCurrentPercent() {
		return getCurrentPosition() / MOVABLE_HEIGHT;
	}

	public double getReferencePosition() {
		return m_controller.getReference();
	}

	public boolean isAtReference(final double tolerance) {
		return epsilonEquals(getCurrentPosition(), getReferencePosition(), tolerance);
	}

	public double getOutput() {
		return m_output;
	}

	@Override
	public void applySafeState() {
		setDisabled();
		m_motor.setPower(0.0);
	}

	@Override
	public void update() {
		if (!m_time.isLaunched()) {
			m_time.launch();
		}

		switch (m_mode) {
			case DISABLED:
				m_output = 0.0;
				break;
			case OPEN_LOOP:
				m_output = m_goal;

				final double percent = getCurrentPercent();

				if (percent < SCRUBBING_RANGE.min && m_output < 0.0) {
					m_output = 0.0;
				} else if (percent > SCRUBBING_RANGE.max && m_output > 0.0) {
					m_output = 0.0;
				}

				break;
			case POSITION:
				m_controller.setReference(m_goal * MOVABLE_HEIGHT);
				m_output = m_controller.update(m_time.update(), m_motor.getCurrentPosition());

				if (m_goal > 0.1) {
					m_output += GRAVITY_FEEDFORWARD;
				}

				break;
		}

		m_motor.setPower(m_output);
	}
}

package com.gemsrobotics.ftc2020.hardware;

import com.gemsrobotics.ftc2020.Subsystem;
import com.gemsrobotics.lib.DeltaTime;
import com.gemsrobotics.lib.controls.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static com.gemsrobotics.lib.utils.MathUtils.epsilonEquals;

public class Extender implements Subsystem {
	private static final double LENGTH = 2000;
	private static final PIDFController.Gains GAINS = new PIDFController.Gains(
			0.04,
			0.0,
			0.0,
			0.0
	);

	protected final DcMotorEx m_motor;
	protected final PIDFController m_controller;
	protected final DeltaTime m_time;

	protected ControlMode m_mode;
	protected double m_goal, m_output;

	public Extender(final HardwareMap hardwareMap) {
		m_motor = hardwareMap.get(DcMotorEx.class, "LiftHorizontalMotor");
		m_motor.setDirection(DcMotorSimple.Direction.FORWARD);
		m_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		m_controller = new PIDFController(GAINS);
		m_controller.setInputRange(0, 2000);
		m_controller.setOutputRange(-0.8, 0.8);
		m_controller.setContinuous(false);
		m_controller.setTolerance(75);
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

	public enum Position {
		STOWED(0.0),
		GRABBING(0.13),
		STOPPED(0.45),
		SCORING(1.00);

		private final double percent;

		Position(final double p) {
			percent = p;
		}
	}

	public void setPositionGoal(final Position position) {
		configureMode(ControlMode.POSITION);
		m_goal = position.percent;
	}

	public void setDisabled() {
		configureMode(ControlMode.DISABLED);
		m_goal = 0.0;
	}

	public double getCurrentPosition() {
		return m_motor.getCurrentPosition();
	}

	public double getCurrentPercent() {
		return m_motor.getCurrentPosition() / LENGTH;
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
				m_motor.setPower(0.0);
				break;
			case OPEN_LOOP:
				m_output = m_goal;
				break;
			case POSITION:
				m_controller.setReference(m_goal * LENGTH);
				m_output = m_controller.update(m_time.update(), m_motor.getCurrentPosition());
				break;
		}

		m_motor.setPower(m_output);
	}
}

package com.gemsrobotics.ftc2020.hardware;

import com.gemsrobotics.ftc2020.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake implements Subsystem {
	private final DcMotorEx m_motorLeft, m_motorRight;
	private Goal m_goal;

	public Intake(final HardwareMap hardwareMap) {
		m_motorLeft = hardwareMap.get(DcMotorEx.class, "IntakeLeftMotor");
		m_motorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
		configureMotor(m_motorLeft);
		m_motorRight = hardwareMap.get(DcMotorEx.class, "IntakeRightMotor");
		m_motorRight.setDirection(DcMotorSimple.Direction.REVERSE);
		configureMotor(m_motorRight);

		m_goal = Goal.DISABLED;
	}

	private void configureMotor(final DcMotorEx motor) {
		motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	}

	public enum Goal {
		DISABLED(0.0),
		NEUTRAL(0.0),
		INTAKING(1.0),
		OUTTAKING(-0.5);

		private final double power;

		Goal(final double p) {
			power = p;
		}
	}

	public void setGoal(final Goal newGoal) {
		if (m_goal != newGoal) {
			if (newGoal == Goal.DISABLED) {
				m_motorLeft.setMotorDisable();
				m_motorRight.setMotorDisable();
			} else {
				m_motorLeft.setMotorEnable();
				m_motorRight.setMotorEnable();
			}

			m_goal = newGoal;
		}
	}

	@Override
	public void applySafeState() {
		setGoal(Goal.DISABLED);
		m_motorLeft.setPower(0.0);
		m_motorRight.setPower(0.0);
	}

	@Override
	public void update() {
		m_motorLeft.setPower(m_goal.power);
		m_motorRight.setPower(m_goal.power);
	}
}

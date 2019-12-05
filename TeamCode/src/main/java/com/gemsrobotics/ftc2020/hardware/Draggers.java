package com.gemsrobotics.ftc2020.hardware;

import com.gemsrobotics.ftc2020.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Draggers implements Subsystem {
	private final Servo m_servoFront, m_servoBack;

	private Goal m_goal;

	public Draggers(final HardwareMap hardwareMap) {
		m_servoFront = hardwareMap.get(Servo.class, "FrontFoundClaw");
		m_servoBack = hardwareMap.get(Servo.class, "BackFoundClaw");
	}

	public enum Goal {
		EXTENDED(1.0, 0.0), RETRACTED(0.0, 1.0);

		private final double positionFront, positionBack;

		Goal(final double p1, final double p2) {
			positionFront = p1;
			positionBack = p2;
		}
	}

	public void setGoal(final Goal goal) {
		m_goal = goal;
	}

	@Override
	public void applySafeState() {
		setGoal(Goal.RETRACTED);
		update();
	}

	@Override
	public void update() {
		m_servoFront.setPosition(m_goal.positionFront);
		m_servoBack.setPosition(m_goal.positionBack);
	}
}

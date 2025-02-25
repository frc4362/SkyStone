package com.gemsrobotics.ftc2020;

import android.text.TextUtils;

import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.gemsrobotics.ftc2020.hardware.Draggers;
import com.gemsrobotics.ftc2020.hardware.Elevator;
import com.gemsrobotics.ftc2020.hardware.Extender;
import com.gemsrobotics.ftc2020.hardware.Intake;
import com.gemsrobotics.ftc2020.hardware.Inventory;
import com.gemsrobotics.ftc2020.hardware.Chassis;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

import static com.gemsrobotics.lib.utils.MathUtils.epsilonEquals;
import static java.lang.StrictMath.abs;

public class Superstructure {
	public static final double
			PASSTHROUGH_FORWARD_POSITION = 1.0,
			PASSTHROUGH_REVERSE_POSITION = 0.0;
	public static final double
			FLIPPER_UPRIGHT_POSITION = 0.8,
			FLIPPER_LEFT_POSITION = 0.4,
			FLIPPER_RIGHT_POSITION = 0.0,
			FLIPPER_STOP_POSITION = 0.6;
	public static final double
			GRIPPER_CLOSED_POSITION = 0.0,
			GRIPPER_OPEN_POSITION = 1.0;
	public static final double
			PARKER_RETRACTED_POSITION = 1.0,
			PARKER_EXTENDED_POSITION = 0.0;
	public static final double
			CAPPER_INIT_POSITION = 0.0,
			CAPPER_DROP_POSITION = 1.0;

	public final Chassis chassis;
	public final Intake intake;
	public final Draggers draggers;
	public final Extender extender;
	public final Elevator elevator;

	public final Inventory inventory;
	public final Servo passthrough;
	public final Servo flipper;
	public final Servo gripper;
	public final Servo parker;
	public final Servo capper;

	private final Telemetry m_telemetry;
	private final List<Subsystem> m_subsystems;
	private final Queue<Request> m_requests;

	private Goal m_newGoal, m_state;

	public Superstructure(final HardwareMap hardwareMap, final Telemetry telemetry) {
		chassis = new Chassis(hardwareMap);
		intake = new Intake(hardwareMap);
		draggers = new Draggers(hardwareMap);
		extender = new Extender(hardwareMap);
		elevator = new Elevator(hardwareMap);

		// not included in the manager
		inventory = new Inventory(hardwareMap);
		passthrough = hardwareMap.get(Servo.class, "Z Rotation");
		passthrough.setDirection(Servo.Direction.FORWARD);
		flipper = hardwareMap.get(Servo.class, "X Rotation");
		flipper.setDirection(Servo.Direction.FORWARD);
		gripper = hardwareMap.get(Servo.class, "Gripper");
		gripper.setDirection(Servo.Direction.FORWARD);
		parker = hardwareMap.get(Servo.class, "ParkExtender");
		parker.setDirection(Servo.Direction.REVERSE);
		capper = hardwareMap.get(Servo.class, "CapServo");

		m_subsystems = Arrays.asList(
				chassis,
				intake,
				draggers,
				extender,
				elevator);
		m_requests = new LinkedList<>();
		m_telemetry = telemetry;
	}

	public enum Goal {
		STOWED,
//		DRAGGING,
		INTAKING,
		SCORING,
		OUTTAKING,
		PARKING,
		GRABBED
	}

	public void request(final Request request) {
		m_requests.add(request);
	}

	public void clearRequests() {
		m_requests.clear();
		setGoal(Goal.STOWED);
	}

	public void update() {
		//  && !(m_state == Goal.GRABBED && m_newGoal == Goal.STOWED)
		if (m_newGoal != null && m_newGoal != m_state) {
			handleGoalTransition(m_newGoal);
			m_newGoal = null;
		}

		for (final Subsystem subsystem : m_subsystems) {
			subsystem.update();
		}

		for (final Iterator<Request> requests = m_requests.iterator(); requests.hasNext();) {
			final Request request = requests.next();
			final boolean isDone = request.execute();

			if (isDone) {
				requests.remove();
			}
		}
	}

	public void setSafeState() {
		for (final Subsystem subsystem : m_subsystems) {
			subsystem.applySafeState();
		}
	}

	public void setGoal(final Goal wantedGoal) {
		m_newGoal = wantedGoal;
	}

	public Goal getState() {
		return m_state;
	}

	public void requestGrab() {
		request(new GrabRequest());
	}

	public void requestDrive(final double distance) {
		request(new DriveStraightRequest(distance));
	}

	public void requestTurn(final Rotation turn) {
		request(new TurnRequest(turn.getRadians()));
	}

	private void handleGoalTransition(final Goal newGoal) {
		switch (newGoal) {
			case STOWED:
				request(new StowRequest(m_state != Goal.GRABBED));
				break;
//			case DRAGGING:
//				request(new StowRequest(true));
//				request(new DragRequest());
//				break;
			case SCORING:
				request(new ScorePositionRequest());
				break;
			case INTAKING:
				request(new IntakingRequest());
				break;
			case OUTTAKING:
//				request(new StowRequest(true));
				request(new OuttakingRequest());
				break;
			case PARKING:
				request(new ParkRequest());
				break;
		}
	}

	public boolean isBusy() {
		return !m_requests.isEmpty();
	}

	public String getRequestSummary() {
		final StringBuilder ret = new StringBuilder();
		ret.append("[");

		final List<String> requestNames = new ArrayList<>();
		for (final Request request : m_requests) {
			requestNames.add(request.getClass().getSimpleName());
		}

		ret.append(TextUtils.join(", ", requestNames));

		ret.append("]");
		return ret.toString();
	}

	public class DriveStraightRequest extends Request {
		private boolean m_hasInitd;
		private double m_distance;

		private DriveStraightRequest(final double distance) {
			m_hasInitd = false;
			m_distance = distance * Chassis.kX;
		}

		@Override
		public boolean execute() {
			if (!m_hasInitd) {
				final TrajectoryBuilder builder = chassis.getTrajectoryBuilder();

				if (m_distance < 0.0) {
					builder.back(abs(m_distance));
				} else {
					builder.forward(abs(m_distance));
				}

				chassis.setTrajectoryGoal(builder.build());
				m_hasInitd = true;
				return false;
			}

			return !chassis.isBusy();
		}
	}

	private class TurnRequest extends Request {
		private boolean m_hasInitd;
		private double m_radians;

		public TurnRequest(final double radians) {
			m_hasInitd = false;
			m_radians = radians;
		}

		@Override
		public boolean execute() {
			if (!m_hasInitd) {
				chassis.setTurnGoal(m_radians);
				m_hasInitd = true;
				return false;
			}

			return !chassis.isBusy();
		}
	}

	private class StowRequest extends Request {
		private boolean moveExtender;
		private long elevatorTime = Long.MAX_VALUE;

		public StowRequest(final boolean moveExtender) {
			this.moveExtender = moveExtender;
		}

		@Override
		public boolean execute() {
			intake.setGoal(Intake.Goal.NEUTRAL);
			gripper.setPosition(GRIPPER_OPEN_POSITION);
			parker.setPosition(PARKER_RETRACTED_POSITION);
			flipper.setPosition(FLIPPER_UPRIGHT_POSITION);

			if (elevator.getCurrentPercent() > 0.05) {
				elevator.setPositionGoal(0.0);
				return false;
			}
			if (elevatorTime == Long.MAX_VALUE) {
				elevatorTime = System.currentTimeMillis() + 1000;
			}

			if (System.currentTimeMillis() < elevatorTime) {
				return false;
			}
			passthrough.setPosition(PASSTHROUGH_FORWARD_POSITION);

			if (extender.getCurrentPercent() > 0.1 && moveExtender) {
				extender.setPositionGoal(Extender.Position.STOWED);
				return false;
			}

			gripper.setPosition(GRIPPER_CLOSED_POSITION);
			m_state = Goal.STOWED;
			return true;
		}
	}

//	private class DragRequest extends Request {
//		@Override
//		public boolean execute() {
//			intake.setGoal(Intake.Goal.NEUTRAL);
//			draggers.setGoal(Draggers.Goal.EXTENDED);
//			parker.setPosition(PARKER_RETRACTED_POSITION);
//			flipper.setPosition(FLIPPER_UPRIGHT_POSITION);
//
//			m_state = Goal.DRAGGING;
//			return true;
//		}
//	}

	private class ScorePositionRequest extends Request {
		private boolean hasCleared = false;
		private long backupTime = Long.MAX_VALUE;
		private long clearTime = Long.MAX_VALUE;

		@Override
		public boolean execute() {
			intake.setGoal(Intake.Goal.NEUTRAL);
			parker.setPosition(PARKER_RETRACTED_POSITION);
			flipper.setPosition(FLIPPER_UPRIGHT_POSITION);

			if (extender.getCurrentPercent() < 0.95 && !hasCleared) {
				extender.setPositionGoal(Extender.Position.CLEARED);
				return false;
			}

			hasCleared = true;

			if (backupTime == Long.MAX_VALUE) {
				backupTime = System.currentTimeMillis() + 250;
			}

			if (System.currentTimeMillis() < backupTime) {
				return false;
			}
			if (clearTime == Long.MAX_VALUE) {
				clearTime = System.currentTimeMillis() + 500;
			}
			if (System.currentTimeMillis() < clearTime) {
				passthrough.setPosition(PASSTHROUGH_REVERSE_POSITION);
				flipper.setPosition(FLIPPER_UPRIGHT_POSITION);
				return false;
			}
			if (extender.getCurrentPercent() > 0.55) {
				extender.setPositionGoal(Extender.Position.SCORE_CLOSE);
				return false;
			}

			m_state = Goal.SCORING;
			return true;
		}
	}

	private class GrabRequest extends Request {
		private long twistTime = Long.MAX_VALUE;
		private long closeTime = Long.MAX_VALUE;

		@Override
		public boolean execute() {
			passthrough.setPosition(PASSTHROUGH_FORWARD_POSITION);
			gripper.setPosition(GRIPPER_CLOSED_POSITION);
			parker.setPosition(PARKER_RETRACTED_POSITION);

			//TODO
//			if (inventory.isCubeTiltedLeft()) {
//				flipper.setPosition(FLIPPER_LEFT_POSITION);
//			} else if (inventory.isCubeTiltedRight()) {
//				flipper.setPosition(FLIPPER_RIGHT_POSITION);
//			} else {
//				flipper.setPosition(FLIPPER_UPRIGHT_POSITION);
//			}

//			if (twistTime == Long.MAX_VALUE) {
//				twistTime = System.currentTimeMillis() + 150;
//			}
//
//			if (twistTime > System.currentTimeMillis()) {
//				return false;
//			}
//
//			if (extender.getCurrentPercent() > 0.25) {
//				extender.setPositionGoal(Extender.Position.GRABBING);
//				return false;
//			}

			gripper.setPosition(GRIPPER_CLOSED_POSITION);
			intake.setGoal(Intake.Goal.NEUTRAL);

			m_state = Goal.GRABBED;
			if (closeTime == Long.MAX_VALUE) {
				closeTime = System.currentTimeMillis() + 200;
			}

			if (closeTime > System.currentTimeMillis()) {
				return false;
			}
			return true;
		}
	}

	private class IntakingRequest extends Request {
		@Override
		public boolean execute() {
			passthrough.setPosition(PASSTHROUGH_FORWARD_POSITION);
//			gripper.setPosition(GRIPPER_CLOSED_POSITION);
			gripper.setPosition(GRIPPER_OPEN_POSITION);
			parker.setPosition(PARKER_RETRACTED_POSITION);

			if (elevator.getCurrentPercent() > 0.03) {
				elevator.setPositionGoal(0.0);
				return false;
			}

//			extender.setPositionGoal(Extender.Position.STOPPED);
			extender.setPositionGoal(Extender.Position.GRABBING);

//			final double currentExtenderPercent = extender.getCurrentPercent();
//
//			if (epsilonEquals(currentExtenderPercent, Extender.Position.STOPPED.percent, 0.05)) {
//				extender.setPositionGoal(Extender.Position.STOPPED);
//				return false;
//			}

//			flipper.setPosition(FLIPPER_STOP_POSITION);
			flipper.setPosition(FLIPPER_UPRIGHT_POSITION);
			intake.setGoal(Intake.Goal.INTAKING);

			m_state = Goal.INTAKING;
			return true;
		}
	}

	private class OuttakingRequest extends Request {
		@Override
		public boolean execute() {
			passthrough.setPosition(PASSTHROUGH_FORWARD_POSITION);
			gripper.setPosition(GRIPPER_OPEN_POSITION);
			parker.setPosition(PARKER_RETRACTED_POSITION);

			if (elevator.getCurrentPercent() > 0.03) {
				elevator.setPositionGoal(0.0);
				return false;
			}

			extender.setPositionGoal(Extender.Position.STOWED);

//			if (extender.getCurrentPercent() > 0.5) {
//				extender.setPositionGoal(Extender.Position.STOWED);
//				return false;
//			}

			intake.setGoal(Intake.Goal.OUTTAKING);

			m_state = Goal.OUTTAKING;
			return true;
		}
	}

	private class ParkRequest extends Request {
		@Override
		public boolean execute() {
			if (extender.getCurrentPercent() < 0.85) {
				extender.setPositionGoal(Extender.Position.CLEARED);
				return false;
			}

			parker.setPosition(PARKER_EXTENDED_POSITION);

			m_state = Goal.PARKING;
			return true;
		}
	}
}

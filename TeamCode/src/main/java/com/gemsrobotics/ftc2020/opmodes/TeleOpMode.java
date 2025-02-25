package com.gemsrobotics.ftc2020.opmodes;

import com.gemsrobotics.ftc2020.Superstructure;
import com.gemsrobotics.ftc2020.hardware.Draggers;
import com.gemsrobotics.ftc2020.hardware.Extender;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static java.lang.StrictMath.abs;

@TeleOp(name="TeleOp Mode")
public final class TeleOpMode extends BaseTeleOpMode {
	private boolean
			wasScrubbingLast,
			wasIncrementingLast,
			wantedStowLast,
			wantedIntakeLast,
			wantedDropCubeLast,
			wantedCapLast;

	@Override
	public void startup() {
		wasScrubbingLast = false;
		wasIncrementingLast = false;
		wantedStowLast = false;
		wantedIntakeLast = false;
		wantedDropCubeLast = false;
		wantedCapLast = false;
	}
	
	@Override
	public void update() {
		final boolean wantsStow = gamepad2.left_bumper;
		final double scrubbingPower = gamepad2.right_trigger - gamepad2.left_trigger;
		final boolean wantsScrub = abs(scrubbingPower) > 0.08;
		final boolean wantsElevatorIncrement = gamepad2.right_bumper;
		final boolean wantsIntake = gamepad2.a;
		final boolean wantsDropCube = gamepad2.y;
		final boolean wantsCap = gamepad2.back;
		final boolean wantsFarScore = gamepad2.dpad_right;
		final boolean wantsCloseGripper = gamepad2.dpad_up;

		superstructure.chassis.setOpenLoopPolar(
				-gamepad1.left_stick_y,
				gamepad1.left_stick_x,
				gamepad1.right_stick_x,
				false,
				gamepad1.left_bumper
		);

		if (wantsStow && !wantedStowLast) {
			superstructure.setGoal(Superstructure.Goal.STOWED);
		} else if (superstructure.getState() == Superstructure.Goal.SCORING
				|| (superstructure.getState() == Superstructure.Goal.PARKING
					&& superstructure.passthrough.getPosition() == Superstructure.PASSTHROUGH_REVERSE_POSITION)
		) {
			// CLEARED LOGIC
			if (wantsScrub) {
				superstructure.elevator.setOpenLoopGoal(scrubbingPower * 0.6);
			} else if (wasScrubbingLast) {
				superstructure.elevator.setHoldPositionGoal();
			} else if (wantsElevatorIncrement && !wasIncrementingLast) {
				superstructure.elevator.incrementPosition();
			}

			if (wantsFarScore){
				superstructure.extender.setPositionGoal(Extender.Position.SCORE_FAR);
			}

			// TODO check phase
			if (wantsDropCube && !wantedDropCubeLast) {
				superstructure.gripper.setPosition(Superstructure.GRIPPER_CLOSED_POSITION);
			} else if (wantedDropCubeLast) {
				superstructure.gripper.setPosition(Superstructure.GRIPPER_OPEN_POSITION);
			}

			if (wantsCloseGripper) {
				superstructure.gripper.setPosition(Superstructure.GRIPPER_CLOSED_POSITION);
			}
		} else if (!superstructure.isBusy() && superstructure.getState() == Superstructure.Goal.GRABBED){
//			if (wantsCap && !wantedCapLast) {
//				superstructure.capper.setPosition(Superstructure.CAPPER_INIT_POSITION);
			if (gamepad2.back) {
				superstructure.capper.setPosition(Superstructure.CAPPER_INIT_POSITION);
			} if (wantsIntake) {
				superstructure.setGoal(Superstructure.Goal.INTAKING);
			} else if (wantedIntakeLast) {
				superstructure.requestGrab();
			} else if (gamepad2.b) {
				superstructure.setGoal(Superstructure.Goal.OUTTAKING);
			} else if (gamepad2.x) {
				superstructure.setGoal(Superstructure.Goal.SCORING);
			}
		} else if (!superstructure.isBusy() && superstructure.getState() != Superstructure.Goal.PARKING) {
			if (wantsIntake) {
				superstructure.setGoal(Superstructure.Goal.INTAKING);
			} else if (wantedIntakeLast) {
				superstructure.requestGrab();
			} else if (gamepad2.b) {
				superstructure.setGoal(Superstructure.Goal.OUTTAKING);
			} else if (gamepad2.x) {
				superstructure.setGoal(Superstructure.Goal.SCORING);
			} else if (gamepad2.dpad_up) {
				superstructure.setGoal(Superstructure.Goal.PARKING);
			} else if (wantsCloseGripper) {
				superstructure.gripper.setPosition(Superstructure.GRIPPER_CLOSED_POSITION);
			} else {
				superstructure.setGoal(Superstructure.Goal.STOWED);
			}
		}

		if (gamepad2.dpad_left) {
			superstructure.draggers.setGoal(Draggers.Goal.EXTENDED);
		} else {
			superstructure.draggers.setGoal(Draggers.Goal.RETRACTED);
		}

		telemetry.addData("Extender Position", superstructure.extender.getCurrentPercent());
		telemetry.addData("Extender Power", superstructure.extender.getOutput());
		telemetry.addData("Requests", superstructure.getRequestSummary());
		telemetry.addData("Superstructure Goal", superstructure.getState());
		telemetry.addData("Pose Estimate", superstructure.chassis.getPoseEstimate());

		wasScrubbingLast = wantsScrub;
		wasIncrementingLast = wantsElevatorIncrement;
		wantedStowLast = wantsStow;
		wantedIntakeLast = wantsIntake;
		wantedDropCubeLast = wantsDropCube;
		wantedCapLast = wantsCap;
	}
}

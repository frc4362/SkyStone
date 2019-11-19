package com.gemsrobotics.ftc2020.opmodes;

import com.gemsrobotics.ftc2020.Superstructure;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static java.lang.StrictMath.abs;

@TeleOp(name="TeleOp Mode")
public final class TeleOpMode extends BaseOpMode {
	private boolean wasScrubbingLast = false;
	private boolean wasIncrementingLast = false;
	private boolean wantedStowLast = false;
	private boolean wantedIntakeLast = false;
	private boolean wantedDropLast = false;

	@Override
	public void update() {
		final boolean wantsStow = gamepad2.dpad_down;
		final double scrubbingPower = gamepad2.right_trigger - gamepad2.left_trigger;
		final boolean isScrubbing = abs(scrubbingPower) > 0.12;
		final boolean isIncrementing = gamepad2.right_bumper;
		final boolean wantsIntake = gamepad2.a;
		final boolean wantsDrop = gamepad2.y;

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
			if (isScrubbing) {
				superstructure.elevator.setOpenLoopGoal(scrubbingPower * 0.6);
			} else if (wasScrubbingLast) {
				superstructure.elevator.setHoldPositionGoal();
			} else if (isIncrementing && !wasIncrementingLast) {
				superstructure.elevator.incrementPosition();
			}

			if (wantsDrop && !wantedDropLast) {
				telemetry.addLine("Set from TeleOpMode:48");
				superstructure.gripper.setPosition(Superstructure.GRIPPER_CLOSED_POSITION);
			} else if (wantedDropLast) {
				telemetry.addLine("Set from TeleOpMode:51");
				superstructure.gripper.setPosition(Superstructure.GRIPPER_OPEN_POSITION);
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
			} else if (gamepad2.left_bumper) {
				superstructure.setGoal(Superstructure.Goal.DRAGGING);
			} else if (gamepad2.dpad_up) {
				superstructure.setGoal(Superstructure.Goal.PARKING);
			} else {
				superstructure.setGoal(Superstructure.Goal.STOWED);
			}
		}

		telemetry.addData("Wants Drop", wantsDrop);
		telemetry.addData("Wants Drop Last", wantedDropLast);
		telemetry.addData("Wants Stow", wantsStow);
		telemetry.addData("Gripper Reference", superstructure.gripper.getPosition());
		telemetry.addData("Requests", superstructure.getRequestSummary());
		telemetry.addData("Superstructure Goal", superstructure.getState());
		telemetry.addData("Pose Estimate", superstructure.chassis.getPoseEstimate());
		telemetry.addData("Left Microswitch", superstructure.inventory.isCubeTiltedLeft());
		telemetry.addData("Right Microswitch", superstructure.inventory.isCubeTiltedRight());

		wasScrubbingLast = isScrubbing;
		wasIncrementingLast = isIncrementing;
		wantedStowLast = wantsStow;
		wantedIntakeLast = wantsIntake;
		wantedDropLast = wantsDrop;
	}
}

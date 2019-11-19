package com.gemsrobotics.ftc2020.opmodes;

import com.gemsrobotics.ftc2020.Superstructure;
import com.gemsrobotics.ftc2020.hardware.Draggers;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class BaseOpMode extends OpMode {
    protected Superstructure superstructure;

    public void initialize() {}

    @Override
    public final void init() {
        superstructure = new Superstructure(hardwareMap, telemetry);
        superstructure.draggers.setGoal(Draggers.Goal.RETRACTED);
        superstructure.passthrough.setPosition(Superstructure.PASSTHROUGH_FORWARD_POSITION);
        superstructure.flipper.setPosition(Superstructure.FLIPPER_UPRIGHT_POSITION);
        initialize();
    }

    public void startup() {}

    @Override
    public final void start() {
        startup();
    }

    public abstract void update();

    @Override
    public final void loop() {
        update();
        superstructure.update();
        telemetry.update();
    }

    public void shutdown() {}

    @Override
    public final void stop() {
        shutdown();
        superstructure.setSafeState();
        // update once to allow for safe-state actuations
        superstructure.update();
    }
}

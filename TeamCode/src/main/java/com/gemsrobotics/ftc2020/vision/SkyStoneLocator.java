package com.gemsrobotics.ftc2020.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public final class SkyStoneLocator {
    private static float mmPerInch = 25.4f;

    private static OpenGLMatrix ROBOT_FROM_CAMERA = OpenGLMatrix
            .translation(7.875f * mmPerInch, 0, 9.0f * mmPerInch)
            .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES, -90, 0, 0));

    private final Telemetry m_telemetry;
    private final VuforiaLocalizer m_vuforia;
    private final VuforiaTrackables m_targets;
    private final VuforiaTrackable m_stoneTarget;

    public SkyStoneLocator(final HardwareMap hardwareMap, final Telemetry telemetry) {
        m_telemetry = telemetry;

        final int cameraMonitorViewId = hardwareMap.appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        final VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VuforiaKey.KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        m_vuforia = ClassFactory.getInstance().createVuforia(parameters);
        m_targets = m_vuforia.loadTrackablesFromAsset("Skystone");
        m_stoneTarget = m_targets.get(0);
        m_stoneTarget.setName("Stone Target");
        m_stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, 2.00f * mmPerInch)
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, 90, 0, -90)));

        ((VuforiaTrackableDefaultListener) m_stoneTarget.getListener()).setPhoneInformation(ROBOT_FROM_CAMERA, parameters.cameraDirection);
    }

    public enum Location {
        NONE, FIRST, SECOND, THIRD
    }

    public void start() {
        m_targets.activate();
    }

    public void stop() {
        m_targets.deactivate();
    }

    public void update() {
        final VuforiaTrackableDefaultListener trackable = ((VuforiaTrackableDefaultListener) m_stoneTarget.getListener());
        final boolean isVisible = trackable.isVisible();

        m_telemetry.addData("Target Visible?", isVisible);

        if (isVisible) {
            final OpenGLMatrix location = trackable.getUpdatedRobotLocation();
            final VectorF translation = location.getTranslation();
            m_telemetry.addData("Target Position", "[%.1f, %.1f, %.1f]",
                    translation.get(0) / mmPerInch,
                    translation.get(1) / mmPerInch,
                    translation.get(2) / mmPerInch);
            Orientation rotation = Orientation.getOrientation(location, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            m_telemetry.addData("Target Orientation", "[%.1f, %.1f, %.1f]",
                    rotation.firstAngle,
                    rotation.secondAngle,
                    rotation.thirdAngle);
        } else {
            m_telemetry.addData("Target Position", "None");
            m_telemetry.addData("Target Orientation", "None");
        }
    }

//    public Location getObservedSkyStoneLocation() {
//        final RelicRecoveryVuMark
//    }
}

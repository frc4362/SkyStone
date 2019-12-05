package com.gemsrobotics.ftc2020.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.Collections;
import java.util.List;

public final class SkyStoneLocator {
    private final Telemetry m_telemetry;
    private final VuforiaLocalizer m_vuforia;
    private final TFObjectDetector m_tf;

    private List<Recognition> m_recognitions;

    public SkyStoneLocator(final HardwareMap hardwareMap, final Telemetry telemetry) {
        m_telemetry = telemetry;

        final int cameraMonitorViewId = hardwareMap.appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        final VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VuforiaKey.KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        m_vuforia = ClassFactory.getInstance().createVuforia(parameters);

        int tfodMonitorViewId = hardwareMap.appContext
                .getResources()
                .getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;

        m_tf = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, m_vuforia);
        m_tf.loadModelFromAsset("Skystone.tflite", "Stone", "Skystone");

        m_recognitions = Collections.emptyList();
    }

    public enum Location {
        UNKNOWN,
        WALL,
        CENTER,
        BRIDGE;

        public static Location classifyTarget(final Recognition target) {
            final double leftEdge = target.getLeft();

            if (leftEdge <= 150) {
                return WALL;
            } else if (leftEdge > 150 && leftEdge <= 630) {
                return CENTER;
            } else if (leftEdge > 630) {
                return BRIDGE;
            }

            return UNKNOWN;
        }
    }

    public void start() {
        if (m_tf != null) {
            m_tf.activate();
        }

        m_telemetry.addLine("Tensorflow initialized");
        m_telemetry.update();
    }

    public void stop() {
        if (m_tf != null) {
            m_tf.shutdown();
        }
    }

    public void update() {
        if (m_tf != null) {
            final List<Recognition> recognitions = m_tf.getRecognitions();
            m_recognitions = recognitions != null ? recognitions : Collections.<Recognition>emptyList();
        }
    }

    public Location getObservedSkyStoneLocation(final RecognitionComparator strategy) {
        if (m_recognitions.size() == 0) {
            return Location.UNKNOWN;
        }

        Collections.sort(m_recognitions, strategy);
        return Location.classifyTarget(m_recognitions.get(0));
    }
}

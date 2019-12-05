package com.gemsrobotics.ftc2020.vision;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.Comparator;

import static java.lang.Math.floor;

public enum RecognitionComparator implements Comparator<Recognition> {
    LEFTMOST {
        @Override
        public int compare(final Recognition r1, final Recognition r2) {
            return (int) (r2.getLeft() - r1.getLeft());
        }
    },
    RIGHTMOST {
        @Override
        public int compare(final Recognition r1, final Recognition r2) {
            return (int) (r1.getRight() - r2.getRight());
        }
    }
}

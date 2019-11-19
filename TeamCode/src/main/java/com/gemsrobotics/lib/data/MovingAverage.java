package com.gemsrobotics.lib.data;

public final class MovingAverage extends LimitedQueue<Double> {
    public MovingAverage(final int sampleSize) {
        super(sampleSize);
    }

    public double getAverage() {
        double ret = 0.0;

        for (double value : this) {
            ret += value;
        }

        return ret / size();
    }
}

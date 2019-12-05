package com.gemsrobotics.lib;

public enum Side {
    LEFT(1), RIGHT(-1);

    public final int turnMultiplier;

    Side(final int multiplier) {
        turnMultiplier = multiplier;
    }
}

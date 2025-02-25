package com.gemsrobotics.lib.math.interpolation;

import com.gemsrobotics.lib.data.InterpolatingTreeMap;

/**
 * A Double that can be interpolated.
 * @see InterpolatingTreeMap
 */
@SuppressWarnings({"unused", "WeakerAccess"})
public class InterpolatingDouble implements
        Interpolatable<InterpolatingDouble>,
        InverseInterpolatable<InterpolatingDouble>,
        Comparable<InterpolatingDouble>
{
    public Double value;

    public InterpolatingDouble(Double val) {
        value = val;
    }

    @Override
    public InterpolatingDouble interpolate(final InterpolatingDouble other, final double x) {
        final double dydx = other.value - value;
        final double searchY = dydx * x + value;
        return new InterpolatingDouble(searchY);
    }

    @Override
    public double inverseInterpolate(final InterpolatingDouble upper, final InterpolatingDouble query) {
        final double upperToLower = upper.value - value;
        if (upperToLower <= 0) {
            return 0;
        }

        final double queryToLower = query.value - value;
        if (queryToLower <= 0) {
            return 0;
        }

        return queryToLower / upperToLower;
    }

    @Override
    public int compareTo(final InterpolatingDouble other) {
        if (other.value < value) {
            return 1;
        } else if (other.value > value) {
            return -1;
        } else {
            return 0;
        }
    }
}

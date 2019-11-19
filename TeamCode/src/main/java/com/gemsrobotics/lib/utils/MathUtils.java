package com.gemsrobotics.lib.utils;

import com.gemsrobotics.lib.math.se2.Translation;

import java.util.Collection;

import static java.lang.Math.*;

@SuppressWarnings("unused")
public class MathUtils {
    public static final double
            Tau = 2 * PI,
            Epsilon = 1e-6;

	private MathUtils() {
	}

	public static double limit(final double v, final double magnitude) {
		return coerce(-magnitude, v, magnitude);
	}

	public static void normalize(final double[] ns, final double limit) {
		// normalize speeds
		double biggestN = abs(ns[0]);

		// start on the second magnitude
		for (int i = 1; i < ns.length; i++) {
			final double temp = abs(ns[i]);
			if (biggestN < temp) {
				biggestN = temp;
			}
		}

		if (biggestN > limit) {
			for (int i = 0; i < ns.length; i++) {
				ns[i] = ns[i] / biggestN;
			}
		}
	}

	public static double lerp(final double a, final double b, double x) {
		x = coerce(0.0, x, 1.0);

        // this slightly algebraically inefficient method maintains precision
        // when a and b are of significantly different magnitudes
        return (a * (1.0 - x)) + (b * x);
	}

	public static double deadband(final double val, final double deadband) {
		if (abs(val) > deadband) {
			return val;
		} else {
			return 0.0;
		}
	}

	// see: https://floating-point-gui.de/errors/comparison/
	public static boolean epsilonEquals(final double a, final double b, final double epsilon) {
		final double absA = abs(a);
		final double absB = abs(b);
		final double diff = abs(a - b);

		if (a == b) { // shortcut, handles infinities
			return true;
		} else if (a == 0 || b == 0 || (absA + absB < Double.MIN_NORMAL)) {
			// a or b is zero or both are extremely close to it
			// relative error is less meaningful here
			return diff < epsilon;
		} else { // use relative error
			return diff / min((absA + absB), Double.MAX_VALUE) < epsilon;
		}
	}

	public static boolean epsilonEquals(final double a, final double b) {
		return epsilonEquals(a, b, Epsilon);
	}

	public static boolean allCloseTo(final Collection<Double> nums, final double value, final double epsilon) {
		for (double num : nums) {
			if (!epsilonEquals(num, value, epsilon)) {
				return false;
			}
		}

		return true;
	}

    public static boolean allCloseTo(final Collection<Double> nums, final double value) {
        return allCloseTo(nums, value, Epsilon);
    }

	public static double coerce(final double bot, double v, final double top) {
        return min(max(v, bot), top);
	}

	public static Translation ellipticalDiscToSquare(final Translation t, final double r) {
		final double u = t.x() / r;
		final double v = t.y() / r;

		return new Translation(
				r * ((sqrt(2 + 2*sqrt(2)*u + u*u - v*v) / 2.0) - (sqrt(2 - 2*sqrt(2)*u + u*u - v*v) / 2.0)),
				r * ((sqrt(2 + 2*sqrt(2)*v - u*u + v*v) / 2.0) - (sqrt(2 - 2*sqrt(2)*v - u*u + v*v) / 2.0))
		);
	}

    public static class Bounds {
        public double min;
        public double max;

        public Bounds(final double min, final double max) {
            this.min = min;
            this.max = max;
        }

        public double coerce(final double val) {
            return MathUtils.coerce(min, val, max);
        }

        public boolean isValid(final double n) {
            return (n <= max) && (n >= min);
        }

        @Override
        public String toString() {
            return "(" + FastDoubleToString.format(min) + ", " + FastDoubleToString.format(max) + ")";
        }
    }
}

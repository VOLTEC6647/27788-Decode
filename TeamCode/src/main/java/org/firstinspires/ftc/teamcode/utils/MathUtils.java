//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package org.firstinspires.ftc.teamcode.utils;

public final class MathUtils {
    private MathUtils() {
        throw new AssertionError("utility class");
    }

    public static int clamp(int value, int low, int high) {
        return Math.max(low, Math.min(value, high));
    }

    public static double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(value, high));
    }

    public static double applyDeadband(double value, double deadband, double maxMagnitude) {
        if (Math.abs(value) > deadband) {
            if (maxMagnitude / deadband > 1.0E12) {
                return value > 0.0 ? value - deadband : value + deadband;
            } else {
                return value > 0.0 ? maxMagnitude * (value - deadband) / (maxMagnitude - deadband) : maxMagnitude * (value + deadband) / (maxMagnitude - deadband);
            }
        } else {
            return 0.0;
        }
    }

    public static double applyDeadband(double value, double deadband) {
        return applyDeadband(value, deadband, 1.0);
    }

    public static double inputModulus(double input, double minimumInput, double maximumInput) {
        double modulus = maximumInput - minimumInput;
        int numMax = (int)((input - minimumInput) / modulus);
        input -= (double)numMax * modulus;
        int numMin = (int)((input - maximumInput) / modulus);
        input -= (double)numMin * modulus;
        return input;
    }

    public static double angleModulus(double angleRadians) {
        return inputModulus(angleRadians, -3.141592653589793, Math.PI);
    }

    public static double interpolate(double startValue, double endValue, double t) {
        return startValue + (endValue - startValue) * clamp(t, 0.0, 1.0);
    }

    public static double inverseInterpolate(double startValue, double endValue, double q) {
        double totalRange = endValue - startValue;
        if (totalRange <= 0.0) {
            return 0.0;
        } else {
            double queryToStart = q - startValue;
            return queryToStart <= 0.0 ? 0.0 : queryToStart / totalRange;
        }
    }

    public static boolean isNear(double expected, double actual, double tolerance) {
        if (tolerance < 0.0) {
            throw new IllegalArgumentException("Tolerance must be a non-negative number!");
        } else {
            return Math.abs(expected - actual) < tolerance;
        }
    }

    public static boolean isNear(double expected, double actual, double tolerance, double min, double max) {
        if (tolerance < 0.0) {
            throw new IllegalArgumentException("Tolerance must be a non-negative number!");
        } else {
            double errorBound = (max - min) / 2.0;
            double error = inputModulus(expected - actual, -errorBound, errorBound);
            return Math.abs(error) < tolerance;
        }
    }
    public static double linear(double x, double inMin, double inMax, double outMin, double outMax) {
        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }
}


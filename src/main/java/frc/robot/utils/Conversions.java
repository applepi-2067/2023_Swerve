package frc.robot.utils;

public class Conversions {
    private static final double DEGREES_PER_REV = 360.0;

    public static double RPMToTicksPer100MS(double RPM, double ticksPerRev) {
        return RPM * ticksPerRev / (60.0 * 10.0);
    }

    public static double ticksToDegrees(double ticks, double ticksPerRev) {
       return (ticks / ticksPerRev) * DEGREES_PER_REV;
    }

    public static double degreesToTicks(double degrees, double ticksPerRev) {
        return (degrees / DEGREES_PER_REV) * ticksPerRev;
    }

    public static double metersPerSecondToRPM(double metersPerSecond, double radiusMeters) {
        return metersPerSecond * 60 / (radiusMeters / (2 * Math.PI));
    }
}

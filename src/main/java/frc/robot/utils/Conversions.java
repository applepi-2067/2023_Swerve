package frc.robot.utils;

public class Conversions {
    private static final double DEGREES_PER_REV = 360.0;

    public static double RPMToTicksPer100MS(double RPM, double ticksPerRev) {
        return RPM * ticksPerRev / (60.0 * 10.0);
    }

    public static double ticksToDegrees(double ticks, double ticksPerRev, double gearRatio) {       // TODO: verify gear ratio math.
       return ticks * (1.0 / ticksPerRev) * (1.0 / gearRatio) * DEGREES_PER_REV;
    }

    public static double degreesToTicks(double degrees, double ticksPerRev, double gearRatio) {
        return degrees * (1.0 / DEGREES_PER_REV) * gearRatio * ticksPerRev;
    }

    public static double metersPerSecondToRPM(double metersPerSecond, double radiusMeters) {
        return metersPerSecond * 60.0 / (radiusMeters  * (2 * Math.PI));
    }

    public static double RPM_ToMetersPerSecond(double RPM, double radiusMeters) {
        return RPM * (1.0 / 60.0) * (2 * Math.PI) * radiusMeters;
    }

    public static double reflectAngle(double degrees) {
        double reflection = 180.0 * -1.0 * Math.signum(degrees);
        return degrees + reflection;
    }

    public static double shiftHalfCircle(double degrees) {
        degrees = degrees % 180.0;
        if (Math.abs(degrees) > 90.0) {
            degrees = reflectAngle(degrees);
        }
        return degrees;
    }

    public static double optimizeTargetAngle(double targetAngle, double currAngle) {
        if (Math.abs(targetAngle - currAngle) <= 90.0) {
            return targetAngle;
        }
        return reflectAngle(targetAngle);
    }
}

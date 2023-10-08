package frc.robot.utils;

public class Conversions {

    public static double RPMToTicksPer100ms(double RPM, double ticksPerRev) {
        return RPM * ticksPerRev / (60.0 * 10.0);
    }

    public static double ticksPer100msToRPM(double ticksPer100ms, double ticksPerRev) {
        return ticksPer100ms * (10.0 * 60.0) * (1.0 / ticksPerRev);
    }

    public static double ticksToMeters(double ticks, double ticksPerRev, double gearRatio, double radiusMeters) {
        return ticks * (1.0 / ticksPerRev) * (1.0 / gearRatio) * (Math.PI * radiusMeters);
    }

    public static double metersPerSecondToRPM(double metersPerSecond, double radiusMeters) {
        return metersPerSecond * 60.0 / (radiusMeters  * (2 * Math.PI));
    }

    public static double rpmToMetersPerSecond(double RPM, double radiusMeters) {
        return RPM * (1.0 / 60.0) * (2 * Math.PI) * radiusMeters;
    }

    public static double motorRotationsToDegrees(double motorRotations, double gearRatio) {
        return (motorRotations / gearRatio) * 360.0;
    }

    public static double degreesToMotorRotations(double degrees, double gearRatio) {
        return (degrees / 360.0) * gearRatio;
    }
}

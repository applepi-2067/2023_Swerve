package frc.robot.utils;

public class Conversions {
    private static final double DEGREES_PER_REV = 360.0;

    public static double rotationsToDegrees(double rotations) {
        return rotations * 360.0;
    }

    public static double degreesToRotations(double degrees) {
        return degrees / 360.0;
    }

    public static double RPMToTicksPer100ms(double RPM, double ticksPerRev) {
        return RPM * ticksPerRev / (60.0 * 10.0);
    }

    public static double ticksPer100msToRPM(double ticksPer100ms, double ticksPerRev) {
        return ticksPer100ms * (10.0 * 60.0) * (1.0 / ticksPerRev);
    }    

    public static double ticksToDegrees(double ticks, double ticksPerRev, double gearRatio) {
       return ticks * (1.0 / ticksPerRev) * (1.0 / gearRatio) * DEGREES_PER_REV;
    }

    public static double degreesToTicks(double degrees, double ticksPerRev, double gearRatio) {
        return degrees * (1.0 / DEGREES_PER_REV) * gearRatio * ticksPerRev;
    }

    public static double metersPerSecondToRPM(double metersPerSecond, double radiusMeters) {
        return metersPerSecond * 60.0 / (radiusMeters  * (2 * Math.PI));
    }

    public static double rpmToMetersPerSecond(double RPM, double radiusMeters) {
        return RPM * (1.0 / 60.0) * (2 * Math.PI) * radiusMeters;
    }
}

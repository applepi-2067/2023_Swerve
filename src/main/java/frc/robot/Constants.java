// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverXBoxControllerPort = 0;
    public static final int kDriverJoystickControllerPort = 1;
  }

  public static class DigitalInputs {
    public static final int GO_CART_SENSOR_DI = 0;
  }

  public static class SwerveModules {
    // Arrays follow the back left to front right convention.

    public static class CAN_IDs {
      public static final int[] DRIVE = {1, 2, 3, 4};
      public static final int[] STEER = {5, 6, 7, 8};
    }

    public static class MINI {
      // Swerve module offsets from center.
      public static final Translation2d[] CENTER_OFFSETS = {
        new Translation2d(Units.inchesToMeters(-9), Units.inchesToMeters(10)),
        new Translation2d(Units.inchesToMeters(-9), Units.inchesToMeters(-10)),
        new Translation2d(Units.inchesToMeters(9), Units.inchesToMeters(10)),
        new Translation2d(Units.inchesToMeters(9), Units.inchesToMeters(-10)),
      };

       // Ticks from absolute sensor zero to wheel zero.
      public static final double[] WHEEL_ZERO_OFFSET_TICKS = {
        -4008.0, 211.0, -684.0, -3063.0
      };
    }

    public static class GO_CART {
      // TODO: why are these offsets like this?
      public static final Translation2d[] CENTER_OFFSETS = {
        new Translation2d(Units.inchesToMeters(-18), Units.inchesToMeters(-11)),
        new Translation2d(Units.inchesToMeters(-18), Units.inchesToMeters(11)),
        new Translation2d(Units.inchesToMeters(18), Units.inchesToMeters(-11)),
        new Translation2d(Units.inchesToMeters(18), Units.inchesToMeters(11)),
      };

      public static final double[] WHEEL_ZERO_OFFSET_TICKS = {
        187.0, -3921.0, -3072.0, 250.0
      };
    }
  }
}

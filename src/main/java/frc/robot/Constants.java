// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

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
    public static final int kDriverControllerPort = 0;
  }

  public static class SwerveModules {
    // Arrays follow the back left to front right convention.

    public static class CAN_IDs {
      public static final int[] DRIVE = {1, 2, 3, 4};
      public static final int[] STEER = {5, 6, 7, 8};
    }

    // Swerve module offsets from center.
    public static final Translation2d[] CENTER_OFFSETS = {        // TODO: find center offsets
      new Translation2d(0.5, 0.5), new Translation2d(0.5, 0.5),
      new Translation2d(0.5, 0.5), new Translation2d(0.5, 0.5)
    };

    // Ticks from absolute sensor zero to wheel zero.
    public static final double[] WHEEL_ZERO_OFFSET_TICKS = {
      -1973.0, -3829.0, -699.0, -1060.0
    };
  }
}

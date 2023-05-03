// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  public static class CAN_IDs {
    public static class SwerveModules {               // TODO: Get actual CAN IDs.
      public static final int BACK_LEFT_DRIVE = 1;
      public static final int BACK_LEFT_STEER = 5;

      public static final int BACK_RIGHT_DRIVE = 2;
      public static final int BACK_RIGHT_STEER = 6;
      
      public static final int FRONT_LEFT_DRIVE = 3;
      public static final int FRONT_LEFT_STEER = 7;

      public static final int FRONT_RIGHT_DRIVE = 4;
      public static final int FRONT_RIGHT_STEER = 8;
    }
  }
}

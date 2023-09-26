package frc.robot;


public final class Constants {
    public static class CAN_IDs {
        public static class Drivetrain {
            // Follows back left to front right convention.
            public static final int[] DRIVE = {1, 2, 3, 4};
            public static final int[] STEER = {5, 6, 7, 8};

            public static final int GYRO = 9;
        }

        public static class Shoulder {
            public static final int MAIN = 10;
            public static final int FOLLOWER = 11;
        }
    }
}

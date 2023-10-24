package frc.robot.constants;


public final class RobotMap {
    public static class canIDs {
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

        public static class Claw {
            public static final int CLAW_GRASP_PNEUMATICS_ID = 0;
            public static final int CLAW_GRASP_PNEUMATICS_CHANNEL = 0;

            public static final int CLAW_BELT = Drivetrain.GYRO;
        }

        public static final int ARM = 12;
    }

    public static class DigitalInputIDs {
        public static final int CLAW_IR_SENSOR = 0;
        public static final int ARM_ZERO_SENSOR = 1;
        public static final int SHOULDER_ZERO_SENSOR = 2;
    }
}

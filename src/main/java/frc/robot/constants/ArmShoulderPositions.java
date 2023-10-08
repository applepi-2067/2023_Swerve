package frc.robot.constants;

import frc.robot.utils.ArmShoulderPosition;


public final class ArmShoulderPositions {
    public static final ArmShoulderPosition STOW = new ArmShoulderPosition(0.0, 0.0);
    public static final ArmShoulderPosition LOW = new ArmShoulderPosition(0.0, -90.0);

    public static class Cube {
        public static final ArmShoulderPosition MID = new ArmShoulderPosition(0.2748, -63.46);
        public static final ArmShoulderPosition HIGH = new ArmShoulderPosition(0.75, -57.5);
    }

    public static class Cone {
        public static final ArmShoulderPosition MID = new ArmShoulderPosition(0.4193, -54.12);
        public static final ArmShoulderPosition HIGH = new ArmShoulderPosition(0.9311, -52.0);
    }

    public static class Pickup {
        public static final ArmShoulderPosition GROUND = new ArmShoulderPosition(0.0, -110.3);
        public static final ArmShoulderPosition DOUBLE_SUBSTATION = new ArmShoulderPosition(0.858, -63.0);
    }
}

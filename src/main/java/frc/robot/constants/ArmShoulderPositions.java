package frc.robot.constants;

import frc.robot.utils.ArmShoulderPosition;


public final class ArmShoulderPositions {
    public static final ArmShoulderPosition STOW = new ArmShoulderPosition(0.0, 0.0);
    public static final ArmShoulderPosition LOW = new ArmShoulderPosition(0.0, -90.0);

    public static class Cube {
        public static final ArmShoulderPosition MID = new ArmShoulderPosition(0.2748 + 0.08, -63.46);
        public static final ArmShoulderPosition HIGH = new ArmShoulderPosition(0.75 + 0.08, -55.5);
    }

    public static class Cone {
        public static final ArmShoulderPosition MID = new ArmShoulderPosition(0.4193 + 0.08, -52.12);
        public static final ArmShoulderPosition HIGH = new ArmShoulderPosition(0.93, -50.0);
    }

    public static class Pickup {
        public static final ArmShoulderPosition GROUND = new ArmShoulderPosition(0.0, -112.0);

        public static final ArmShoulderPosition DOUBLE_SUBSTATION = new ArmShoulderPosition(0.858, -63.0);
        public static final ArmShoulderPosition SINGLE_SUBSTATION = new ArmShoulderPosition(0.0, -46.0);
    }
}

package frc.robot.constants;

import frc.robot.utils.ScoringPosition;


public final class ScoringPositions {
    public static final ScoringPosition STOW = new ScoringPosition(0.0, 0.0);
    public static final ScoringPosition LOW = new ScoringPosition(0.0, -90.0);

    public static class CUBE {
        public static final ScoringPosition MID = new ScoringPosition(0.2748, -63.46);
        public static final ScoringPosition HIGH = new ScoringPosition(0.7346, -60.27);
    }

    public static class CONE {
        public static final ScoringPosition MID = new ScoringPosition(0.4193, -54.12);
        public static final ScoringPosition HIGH = new ScoringPosition(0.9311, -52.0);
    }
}

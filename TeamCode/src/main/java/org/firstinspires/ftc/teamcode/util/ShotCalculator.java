package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;

public final class ShotCalculator {

    private static final double HIGH_GOAL_MIN_Y = 70.0;

    private ShotCalculator() {}

    public static final class Solution {
        public final Pose goalPose;
        public final Pose turretPose;
        public final double turretAngle;
        public final double distance;

        private Solution(Pose goalPose, Pose turretPose,
                         double turretAngle, double distance) {
            this.goalPose = goalPose;
            this.turretPose = turretPose;
            this.turretAngle = turretAngle;
            this.distance = distance;
        }
    }

    public static Solution calculate(Pose robotPose, Constants.Alliance alliance) {
        // Select by robot position, independently of turret offset or heading.
        boolean highGoal = robotPose.y() >= HIGH_GOAL_MIN_Y;
        Pose goalPose;
        if (alliance == Constants.Alliance.RED) {
            goalPose = highGoal
                    ? Constants.ShotCalculator.redGoalHigh
                    : Constants.ShotCalculator.redGoalLow;
        } else {
            goalPose = highGoal
                    ? Constants.ShotCalculator.blueGoalHigh
                    : Constants.ShotCalculator.blueGoalLow;
        }

        Pose turretPose = turretPosition(robotPose);

        double dx = goalPose.x() - turretPose.x();
        double dy = goalPose.y() - turretPose.y();

        double turretAngle = Angle.normalizeSigned(
                Math.atan2(dy, dx) - robotPose.heading()
        );

        double distance = Math.hypot(dx, dy);

        return new Solution(goalPose, turretPose, turretAngle, distance);
    }

    public static Pose turretPosition(Pose robotPose) {
        Pose offset = Constants.Turret.turretOffset;

        double cos = Math.cos(robotPose.heading());
        double sin = Math.sin(robotPose.heading());

        return new Pose(
                robotPose.x() + offset.x() * cos - offset.y() * sin,
                robotPose.y() + offset.x() * sin + offset.y() * cos,
                robotPose.heading()
        );
    }
}

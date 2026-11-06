package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;

/** Auto-to-TeleOp data, kept only while the Robot Controller app is running. */
public final class MatchState {
    private static Pose pose = null;
    private static Constants.Alliance alliance = null;

    private MatchState() {}

    public static void save(Pose currentPose, Constants.Alliance currentAlliance) {
        pose = new Pose(currentPose.x(), currentPose.y(), currentPose.heading());
        alliance = currentAlliance;
    }

    public static boolean hasState() { return pose != null; }
    public static boolean hasState(Constants.Alliance selectedAlliance) {
        return pose != null && alliance == selectedAlliance;
    }
    public static Pose getPose() { return pose; }

    public static void clear() {
        pose = null;
        alliance = null;
    }
}

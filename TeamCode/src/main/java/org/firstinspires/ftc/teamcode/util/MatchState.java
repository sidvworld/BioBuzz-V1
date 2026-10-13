package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;

/** Auto-to-TeleOp data, kept only while the Robot Controller app is running. */
public final class MatchState {
    private static Pose pose;

    private MatchState() {}

    public static void save(Pose currentPose) {
        pose = new Pose(currentPose.x(), currentPose.y(), currentPose.heading());
    }

    public static boolean hasState() { return pose != null; }
    public static Pose getPose() { return pose; }

    public static void clear() {
        pose = null;
    }
}

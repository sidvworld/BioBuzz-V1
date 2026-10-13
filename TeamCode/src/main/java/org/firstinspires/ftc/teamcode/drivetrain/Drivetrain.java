package org.firstinspires.ftc.teamcode.drivetrain;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public class Drivetrain {
    public enum DrivetrainState {
        OFF,
        MANUAL,
        SLOW,
        HOLD,
        FOLLOWING
    }

    private static final double DRIVE_SCALE = 1.0;
    private static final double SLOW_SCALE = 0.5;
    private static final double DEADBAND = 0.05;

    private final Follower follower;
    private final Gamepad gamepad;

    private DrivetrainState drivetrainState = DrivetrainState.OFF;
    private DrivetrainState previousState;
    private Path currentPath;

    public Drivetrain(HardwareMap hardwareMap, Gamepad gamepad) {
        follower = Constants.create(hardwareMap);
        this.gamepad = gamepad;
    }

    public Command setState(DrivetrainState state) {
        if (state == null || state == DrivetrainState.FOLLOWING) {
            throw new IllegalArgumentException("?");
        }

        return instant(() -> drivetrainState = state);
    }

    public Command startPathCmd(Path path) {
        if (path == null) {
            throw new IllegalArgumentException("?");
        }

        return instant(() -> {
            currentPath = path;
            drivetrainState = DrivetrainState.FOLLOWING;
            previousState = null;
        });
    }

    private void enterState(DrivetrainState state) {
        switch (state) {
            case HOLD:
                follower.hold(follower.pose());
                break;

            case FOLLOWING:
                follower.follow(currentPath);
                break;

            case OFF:
            case MANUAL:
            case SLOW:
                break;
        }
    }

    private void cycle() {
        if (drivetrainState != previousState) {
            enterState(drivetrainState);
            previousState = drivetrainState;
        }

        switch (drivetrainState) {
            case OFF:
                manualDrive(0.0);
                break;
            case MANUAL:
                manualDrive(DRIVE_SCALE);
                break;

            case SLOW:
                manualDrive(SLOW_SCALE);
                break;

            case HOLD:
            case FOLLOWING:
                break;
        }
    }

    private static double deadband(double value) {
        if (Math.abs(value) <= DEADBAND) return 0.0;

        return Math.copySign((Math.abs(value) - DEADBAND) / (1.0 - DEADBAND), value);
    }

    private void manualDrive(double scale) {
        follower.manual(
                deadband(-gamepad.left_stick_y) * scale,
                deadband(-gamepad.left_stick_x) * scale,
                deadband(-gamepad.right_stick_x) * scale
        );
    }

    public Pose getPose() {
        return follower.pose();
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    public DrivetrainState getState() {
        return drivetrainState;
    }

    public void periodic() {
        cycle();
        follower.update();
        if (drivetrainState == DrivetrainState.FOLLOWING && !follower.isBusy()) {
            drivetrainState = DrivetrainState.HOLD;
            previousState = DrivetrainState.HOLD;
        }
    }
}

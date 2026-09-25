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

    public final Follower follower;
    private final Gamepad gamepad;

    private DrivetrainState drivetrainState;
    private DrivetrainState previousState;

    private Path currentPath;
    private Pose holdPose;

    public enum DrivetrainState {
        MANUAL,
        SLOW,
        HOLD,
        FOLLOWING
    }

    private static final double DRIVE_SCALE = 1.0;
    private static final double SLOW_SCALE = 0.5;

    public Drivetrain(HardwareMap hardwareMap, Gamepad gamepad) {
        follower = Constants.create(hardwareMap);
        this.gamepad = gamepad;

        drivetrainState = DrivetrainState.MANUAL;
        previousState = null;
    }

    public Command setState(DrivetrainState state) {
        return instant(() -> drivetrainState = state);
    }

    public void cycle() {
        if (drivetrainState != previousState) {
            enterState(drivetrainState);
            previousState = drivetrainState;
        }

        switch (drivetrainState) {
            case MANUAL:
                manualDrive(DRIVE_SCALE);
                break;
            case SLOW:
                manualDrive(SLOW_SCALE);
                break;
            case HOLD:
                break;
            case FOLLOWING:
                if (!follower.isBusy()) {
                    drivetrainState = DrivetrainState.MANUAL;
                }
                break;
        }
    }

    private void enterState(DrivetrainState state) {
        switch (state) {
            case MANUAL:
                break;

            case SLOW:
                break;
            case HOLD:
                holdPose = follower.pose();
                follower.hold(holdPose);
                break;
            case FOLLOWING:
                if (currentPath != null) {
                    follower.follow(currentPath);
                }
                break;
        }
    }

    private void manualDrive(double scale) {
        follower.manual(
                -gamepad.left_stick_y * scale,
                gamepad.left_stick_x * scale,
                gamepad.right_stick_x * scale
        );
    }

    public Pose getPose() {
        return follower.pose();
    }

    public DrivetrainState getState() {
        return drivetrainState;
    }

    public void periodic() {
        cycle();
        follower.update();
    }
}
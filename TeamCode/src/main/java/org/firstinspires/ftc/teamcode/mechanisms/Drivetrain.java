package org.firstinspires.ftc.teamcode.mechanisms;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;

import static org.firstinspires.ftc.teamcode.util.Constants.Drivetrain.DEADBAND;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Constants;

public class Drivetrain {
    public enum DrivetrainState {
        OFF,
        MANUAL,
        SLOW,
        HOLD,
        FOLLOWING
    }

    private final Follower follower;
    private final Gamepad gamepad;
    private final Constants.Alliance alliance;

    private DrivetrainState drivetrainState = DrivetrainState.OFF;
    private DrivetrainState previousState;
    private Path currentPath;

    public Drivetrain(HardwareMap hardwareMap, Gamepad gamepad, Constants.Alliance alliance) {
        follower = org.firstinspires.ftc.teamcode.util.Constants.create(hardwareMap);
        this.gamepad = gamepad;
        this.alliance = alliance;
    }

    public Command setState(DrivetrainState state) {
        if (state == DrivetrainState.FOLLOWING) {
            return instant(() -> {});
        }
        return instant(() -> drivetrainState = state);
    }

    public Command startPathCmd(Path path) {
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
                manualDrive(Constants.Drivetrain.DRIVE_SCALE);
                break;

            case SLOW:
                manualDrive(Constants.Drivetrain.SLOW_SCALE);
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

    // field centric
    private void manualDrive(double scale) {
        // Forward: blue -X, red +X. Left: -Y for both alliances.
        double forwardSign = alliance == Constants.Alliance.RED ? 1.0 : -1.0;
        double fieldX = deadband(-gamepad.left_stick_y) * scale * forwardSign;
        double fieldY = deadband(gamepad.left_stick_x) * scale;
        double heading = follower.pose().heading();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        follower.manual(
                fieldX * cos + fieldY * sin,
                -fieldX * sin + fieldY * cos,
                deadband(-gamepad.right_stick_x) * scale
        );
    }

    // robot-centric
    /*
    private void manualDrive(double scale) {
        follower.manual(
                deadband(-gamepad.left_stick_y) * scale,
                deadband(-gamepad.left_stick_x) * scale,
                deadband(-gamepad.right_stick_x) * scale
        );
    }
    */

    public Command waitUntilPathFinished(){
        return waitUntil(() ->
                drivetrainState != DrivetrainState.FOLLOWING
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
        }
    }
}

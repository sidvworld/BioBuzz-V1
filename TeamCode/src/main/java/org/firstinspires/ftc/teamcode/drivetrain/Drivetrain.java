package org.firstinspires.ftc.teamcode.drivetrain;

import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.hold;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public class Drivetrain {

    public final Follower follower;

    public Drivetrain(HardwareMap hardwareMap) {
        follower = Constants.create(hardwareMap);
    }

    public void update() {
        follower.update();
    }

    public void manualDrive(Gamepad gamepad) {
        follower.manual(
                -gamepad.left_stick_y,
                gamepad.left_stick_x,
                gamepad.right_stick_x
        );
    }

    public Command followPath(Path path) {
        return follow(follower, path);
    }

    public Command holdPosition() {
        return hold(follower);
    }

    public Command holdPosition(Pose pose) {
        return hold(follower, pose);
    }

    public Follower getFollower() {
        return follower;
    }

    public Pose getPose() {
        return follower.pose();
    }

    public boolean isFollowing() {
        return follower.following();
    }
}
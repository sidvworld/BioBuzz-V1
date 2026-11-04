package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.MatchState;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;

@NextTeleop(name = "Testing Teleop")
public class TestingTeleop extends NextOpMode {
    private final Robot robot;

    public TestingTeleop(Robot robot) {
        super(robot);
        this.robot = robot;
        robot.alliance = Constants.Alliance.BLUE;
        robot.init(this, false);
        if (MatchState.hasState()) {
            robot.drivetrain.setPose(MatchState.getPose());
        } else { robot.drivetrain.setPose(new Pose(7, 7, Math.toRadians(90))); }
    }

    @Override
    public void start() {
        robot.setState(Robot.State.IDLE);
    }

    @Override
    public void periodic() {
        MatchState.clear();
    }
}

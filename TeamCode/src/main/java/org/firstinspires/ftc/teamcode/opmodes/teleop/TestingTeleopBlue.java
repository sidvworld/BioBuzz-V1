package org.firstinspires.ftc.teamcode.opmodes.teleop;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.MatchState;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;

@NextTeleop(name = "Testing Teleop Blue")
public class TestingTeleopBlue extends NextOpMode {
    private final Robot robot;

    public TestingTeleopBlue(Robot robot) {
        this(robot, Constants.Alliance.BLUE);
    }

    protected TestingTeleopBlue(Robot robot, Constants.Alliance alliance) {
        super(robot);
        this.robot = robot;
        robot.alliance = alliance;
        robot.init(this, false);

        if (MatchState.hasState(alliance)) {
            robot.drivetrain.setPose(MatchState.getPose());
        } else {
            robot.drivetrain.setPose(Constants.Drivetrain.START_POSE);
        }
    }

    @Override
    public void start() {
        robot.setState(Robot.State.IDLE).schedule();
    }

    @Override
    public void periodic() {
        MatchState.clear();
    }
}

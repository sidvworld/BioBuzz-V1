package org.firstinspires.ftc.teamcode.opmodes.teleop;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.StateMachine;
import org.firstinspires.ftc.teamcode.util.MatchState;
import org.firstinspires.ftc.teamcode.util.Alliance;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;

@NextTeleop(name = "Testing")
public class Testing extends NextOpMode {
    private final Robot robot;

    public Testing(Robot robot) {
        super(robot);
        this.robot = robot;
        robot.alliance = Alliance.BLUE;
        robot.init(this, false);
        if (MatchState.hasState()) {
            robot.drivetrain.setPose(MatchState.getPose());
        }
    }

    @Override
    public void start() {
        robot.fsm.requestState(StateMachine.State.IDLE);
    }

    @Override
    public void periodic() {
        MatchState.clear();
    }
}

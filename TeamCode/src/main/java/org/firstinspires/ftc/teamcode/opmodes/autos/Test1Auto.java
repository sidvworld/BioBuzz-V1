package org.firstinspires.ftc.teamcode.opmodes.autos;

import org.firstinspires.ftc.teamcode.opmodes.autos.routines.AutoRoutines;
import org.firstinspires.ftc.teamcode.robot.Robot;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.MatchState;

import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Test1 Auto")
public class Test1Auto extends NextOpMode {
    private final Robot robot;

    public Test1Auto(Robot robot) {
        super(robot);
        this.robot = robot;

        robot.alliance = Constants.Alliance.BLUE;
        robot.init(this, true);

        robot.drivetrain.setPose(AutoRoutines.Test1.test1start);
    }

    @Override
    public void start() {
        robot.setState(Robot.State.IDLE);
    }

    @Override
    public void end() {
        MatchState.save(robot.drivetrain.getPose());
    }
}
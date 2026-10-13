package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.auto_util.AutoRoutines;
import org.firstinspires.ftc.teamcode.util.MatchState;
import org.firstinspires.ftc.teamcode.util.Alliance;

import dev.nextftc.robot.opmode.NextAutonomous;
import dev.nextftc.robot.opmode.NextOpMode;

@NextAutonomous(name = "Test1")
public class Test1 extends NextOpMode {
    private final Robot robot;

    public Test1(Robot robot) {
        super(robot);
        this.robot = robot;
        robot.alliance = Alliance.BLUE;
        MatchState.clear();
        robot.init(this, true);
        robot.drivetrain.setPose(AutoRoutines.Test1.test1start);
    }

    @Override
    public void start(){
        AutoRoutines.Test1.run(robot).schedule();
    }

    @Override
    public void periodic() {
        MatchState.save(robot.drivetrain.getPose());
    }

    @Override
    public void end() {
        MatchState.save(robot.drivetrain.getPose());
    }

}
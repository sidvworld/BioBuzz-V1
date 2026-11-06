package org.firstinspires.ftc.teamcode.opmodes.teleop;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;

import dev.nextftc.robot.opmode.NextTeleop;

@NextTeleop(name = "Testing Teleop Red")
public class TestingTeleopRed extends TestingTeleopBlue {
    public TestingTeleopRed(Robot robot) {
        super(robot, Constants.Alliance.RED);
    }
}

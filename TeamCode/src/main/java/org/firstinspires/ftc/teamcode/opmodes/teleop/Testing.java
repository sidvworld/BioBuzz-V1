package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;

@NextTeleop(name = "Testing")
public class Testing extends NextOpMode {
    public Testing() {
        super(new Robot(Alliance.BLUE));
        Scheduler.reset();
    }

    @Override
    public void periodic() {
        super.periodic();
        Scheduler.execute();
    }
}
package org.firstinspires.ftc.teamcode.opmodes.auto;

import androidx.annotation.NonNull;

import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.auto_util.AutoRoutines;

import java.lang.annotation.Annotation;

import dev.nextftc.robot.opmode.NextAutonomous;
import dev.nextftc.robot.opmode.NextOpMode;

@NextAutonomous(name = "Test1")
public class Test1 extends NextOpMode {
    public Test1(Robot robot) {
        super(new Robot(Alliance.BLUE));
        Scheduler.reset();
    }

    @Override
    public void start(){
        AutoRoutines.Test1.run(super).schedule();
    }

    @Override
    public void periodic(){
        super.periodic();
        Scheduler.execute();
    }

}
package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.data_util.Alliance;

@TeleOp (name = "Testing")
public class Testing extends OpMode {

    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(gamepad1, Alliance.BLUE);
        robot.init();
    }

    @Override
    public void loop() {
        robot.periodic();
    }
}
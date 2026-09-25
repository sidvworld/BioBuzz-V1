package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerLog;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.data_util.GamepadData;
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.data_util.Alliance;
import org.firstinspires.ftc.teamcode.mechanisms.Transfer;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import dev.nextftc.robot.NextRobot;
import dev.nextftc.robot.triggers.CommandGamepad;
public class Robot implements NextRobot {


    public final Gamepad gamepad;
    public final CommandGamepad gp1;
    public final Alliance alliance;

    public final Intake intake;
    public final Shooter shooter;
    public final Transfer transfer;
    public final Drivetrain drivetrain;
    public Robot(Gamepad gamepad, Alliance alliance) {
        this.gamepad = gamepad;
        gp1 = new CommandGamepad(gamepad);
        this.alliance = alliance;

        intake = new Intake();
        shooter = new Shooter();
        transfer = new Transfer();
        drivetrain = new Drivetrain(hardwareMap);
    }

    public void configureKeybinds(){
        // intake
        gp1.leftTrigger().isUnder(GamepadData.leftTriggerThreshold).onTrue(intake.idle());
        gp1.leftTrigger().isOver(GamepadData.leftTriggerThreshold).onTrue(intake.run());
        gp1.a().onTrue(intake.toggleDirection());

        // shooter
        gp1.rightTrigger().isUnder(GamepadData.rightTriggerThreshold).onTrue(shooter.idle());
        gp1.rightTrigger().isOver(GamepadData.rightTriggerThreshold).onTrue(
                parallel(
                        shooter.run(),
                        sequential(waitUntil(shooter::isReady), transfer.run())
                )
        );
        // gp1.rightTrigger().isOver(GamepadData.rightTriggerThreshold).onTrue(shooter.run())
        gp1.b().onTrue(shooter.toggleBall());
    }
}

package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.teamcode.data_util.GamepadData;
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.data_util.Alliance;
import org.firstinspires.ftc.teamcode.mechanisms.Transfer;

import dev.nextftc.robot.NextRobot;
import dev.nextftc.robot.triggers.CommandGamepad;
public class Robot implements NextRobot {
    public final CommandGamepad gp1;
    public final Alliance alliance;

    public final Intake intake;
    public final Shooter shooter;
    public final Transfer transfer;
    public final Drivetrain drivetrain;
    public Robot(Alliance alliance) {
        gp1 = new CommandGamepad(gamepad1);
        this.alliance = alliance;

        intake = new Intake();
        shooter = new Shooter();
        transfer = new Transfer();
        drivetrain = new Drivetrain(hardwareMap, gamepad1);
    }

    public void configureKeybinds(){
        // intake
        gp1.leftTrigger().isUnder(GamepadData.leftTriggerThreshold).onTrue(intake.setState(Intake.IntakeState.IDLE));
        gp1.leftTrigger().isOver(GamepadData.leftTriggerThreshold).onTrue(intake.setState(Intake.IntakeState.RUNNING));
        gp1.a().onTrue(intake.toggleDirection());

        // shooter
        // pollen
        gp1.rightTrigger().isUnder(GamepadData.rightTriggerThreshold).onTrue(parallel(
                drivetrain.setState(Drivetrain.DrivetrainState.MANUAL),
                shooter.setState(Shooter.ShooterState.IDLE),
                transfer.setState(Transfer.TransferState.IDLE)
        ));
        gp1.rightTrigger().isOver(GamepadData.rightTriggerThreshold).onTrue(
                parallel(
                        drivetrain.setState(Drivetrain.DrivetrainState.HOLD),
                        shooter.setState(Shooter.ShooterState.START_POLLEN),
                        sequential(waitUntil(shooter::isReady), transfer.setState(Transfer.TransferState.RUNNING))
                ));
        // nectar
        gp1.rightBumper().onFalse(parallel(
                drivetrain.setState(Drivetrain.DrivetrainState.MANUAL),
                shooter.setState(Shooter.ShooterState.IDLE),
                transfer.setState(Transfer.TransferState.IDLE)
        ));
        gp1.rightBumper().onTrue(
                parallel(
                        drivetrain.setState(Drivetrain.DrivetrainState.HOLD),
                        shooter.setState(Shooter.ShooterState.START_NECTAR),
                        sequential(waitUntil(shooter::isReady), transfer.setState(Transfer.TransferState.RUNNING))
                ));
        // gp1.rightTrigger().isOver(GamepadData.rightTriggerThreshold).onTrue(shooter.run())
    }

    public void dataLog(){
        telemetry.addData("Outtake RPM Setpoint: ", Shooter.VELOCITY_SETPOINT);
        telemetry.addData("Outtake RPM: ", shooter.getRPM());
        telemetry.addData("Met Threshold: ", shooter.isReady());
        telemetry.update();
    }

    public void init(){
        drivetrain.setState(Drivetrain.DrivetrainState.MANUAL);
        configureKeybinds();
        dataLog();
    }

    public void periodic(){
        drivetrain.periodic();
        shooter.periodic();
        intake.periodic();
        transfer.periodic();
        dataLog();
    }
}


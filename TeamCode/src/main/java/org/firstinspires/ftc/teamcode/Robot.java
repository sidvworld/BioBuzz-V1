package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.ivy.Command;

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
    public StateMachine fsm = new StateMachine();
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

    private Command idleMechanisms() {
        return parallel(
                intake.setState(Intake.IntakeState.IDLE),
                shooter.setState(Shooter.ShooterState.IDLE),
                transfer.setState(Transfer.TransferState.IDLE)
        );
    }

    private Command offMechanisms() {
        return parallel(
                drivetrain.setState(Drivetrain.DrivetrainState.OFF),
                intake.setState(Intake.IntakeState.OFF),
                shooter.setState(Shooter.ShooterState.OFF),
                transfer.setState(Transfer.TransferState.OFF)
        );
    }

    private void configureFSM() {
        fsm.configureState(
                StateMachine.State.OFF,
                this::offMechanisms,
                StateMachine::none,
                StateMachine::none
        );

        fsm.configureState(
                StateMachine.State.IDLE,
                this::idleMechanisms,
                StateMachine::none,
                StateMachine::none
        );

        fsm.configureState(
                StateMachine.State.INTAKE,
                () -> intake.setState(Intake.IntakeState.RUNNING),
                StateMachine::none,
                () -> intake.setState(Intake.IntakeState.IDLE)
        );

        fsm.configureState(
                StateMachine.State.SHOOT_POLLEN,
                () -> parallel(
                        shooter.setState(Shooter.ShooterState.REVERSE),
                        transfer.setState(Transfer.TransferState.REVERSE)
                ),
                () -> sequential(
                        waitMs(200),
                        idleMechanisms(),
                        parallel(
                                drivetrain.setState(Drivetrain.DrivetrainState.HOLD),
                                shooter.setState(Shooter.ShooterState.START_POLLEN),
                                sequential(
                                        waitUntil(shooter::isReady),
                                        transfer.setState(Transfer.TransferState.RUNNING)
                                )
                        )
                ),
                this::idleMechanisms
        );

        fsm.configureState(
                StateMachine.State.INTAKE_AND_SHOOT,
                () -> parallel(
                        shooter.setState(Shooter.ShooterState.REVERSE),
                        transfer.setState(Transfer.TransferState.REVERSE),
                        intake.setState(Intake.IntakeState.RUNNING)
                ),
                () -> sequential(
                        waitMs(300),
                        transfer.setState(Transfer.TransferState.IDLE),
                        parallel(
                                drivetrain.setState(Drivetrain.DrivetrainState.HOLD),
                                shooter.setState(Shooter.ShooterState.START_POLLEN),
                                sequential(
                                        waitUntil(shooter::isReady),
                                        transfer.setState(Transfer.TransferState.RUNNING)
                                )
                        )
                ),
                this::idleMechanisms
        );
    }

    private void configureKeybinds() {
        gp1.b().toggleOnTrue(fsm.requestStateCmd(StateMachine.State.OFF));
        gp1.b().toggleOnFalse(fsm.requestStateCmd(StateMachine.State.IDLE));

        gp1.rightBumper()
                .and(gp1.leftBumper())
                .onTrue(fsm.requestStateCmd(StateMachine.State.INTAKE_AND_SHOOT));

        gp1.rightBumper()
                .and(gp1.leftBumper().negate())
                .onTrue(fsm.requestStateCmd(StateMachine.State.SHOOT_POLLEN));

        gp1.leftBumper()
                .and(gp1.rightBumper().negate())
                .onTrue(fsm.requestStateCmd(StateMachine.State.INTAKE));

        gp1.leftBumper()
                .or(gp1.rightBumper())
                .negate()
                .onTrue(fsm.requestStateCmd(StateMachine.State.IDLE));
    }

    public void dataLog(){
        telemetry.addData("Outtake RPM Setpoint: ", Shooter.SHOOTING_SETPOINT);
        telemetry.addData("Outtake RPM: ", shooter.getRPM());
        telemetry.addData("Met Threshold: ", shooter.isReady());
        telemetry.update();
    }

    public void init(){
        configureFSM();
        configureKeybinds();

        fsm.requestStateCmd(StateMachine.State.IDLE);
        dataLog();
    }

    public void periodic(){
        fsm.update();
        drivetrain.periodic();
        shooter.periodic();
        intake.periodic();
        transfer.periodic();
        dataLog();
    }
}

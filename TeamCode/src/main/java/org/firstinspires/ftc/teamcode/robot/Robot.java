package org.firstinspires.ftc.teamcode.robot;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.mechanisms.Transfer;

import dev.nextftc.robot.NextRobot;
import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.triggers.CommandGamepad;
public class Robot implements NextRobot {
    public CommandGamepad gp1;
    private boolean autonomous;
    private boolean off;
    public Alliance alliance;
    public StateMachine fsm = new StateMachine();
    public Intake intake;
    public Shooter shooter;
    public Transfer transfer;
    public Drivetrain drivetrain;
    public Robot() {}

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

    private Command feedWhenReady() {
        return transfer.infinite(() -> transfer.transferState = shooter.isReady() ? Transfer.TransferState.RUNNING : Transfer.TransferState.IDLE);
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
                () -> autonomous ? idleMechanisms() : parallel(
                        idleMechanisms(), drivetrain.setState(Drivetrain.DrivetrainState.MANUAL)),
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
                                feedWhenReady()
                        )
                ),
                () -> parallel(idleMechanisms(), drivetrain.setState(Drivetrain.DrivetrainState.MANUAL))
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
                                feedWhenReady()
                        )
                ),
                () -> parallel(idleMechanisms(), drivetrain.setState(Drivetrain.DrivetrainState.MANUAL))
        );
    }

    private void configureKeybinds() {
        gp1.b().onTrue(instant(() -> {
            off = !off;
            fsm.requestState(off ? StateMachine.State.OFF : StateMachine.State.IDLE);
        }));

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
        Pose pose = drivetrain.getPose();
        Telemetry.log("robot state: ", fsm.getState());
        Telemetry.log("dt State: ", drivetrain.getState());
        Telemetry.log("Pose X: ", pose.x());
        Telemetry.log("Pose Y: ", pose.y());
        Telemetry.log("Heading: ", Math.toDegrees(pose.heading()));
        Telemetry.log("Outtake RPM Setpoint: ", shooter.getRPMSetpoint());
        Telemetry.log("Outtake RPM: ", shooter.getRPM());
        Telemetry.log("Shooter Ready: ", shooter.isReady());
    }

    public void init(NextOpMode opMode, boolean autonomous){
        this.autonomous = autonomous;
        off = false;
        gp1 = new CommandGamepad(opMode.gamepad1);
        intake = new Intake();
        shooter = new Shooter();
        transfer = new Transfer();
        drivetrain = new Drivetrain(opMode.hardwareMap, opMode.gamepad1);
        configureFSM();
        if (!autonomous) configureKeybinds();

        if (!autonomous) {
            intake.intakeState = Intake.IntakeState.OFF;
            shooter.shooterState = Shooter.ShooterState.OFF;
            shooter.IS_READY = false;
            transfer.transferState = Transfer.TransferState.OFF;
        }
        fsm.initialize(autonomous ? StateMachine.State.IDLE : StateMachine.State.OFF);
        dataLog();
    }

    public void periodic(){
        if (off) fsm.requestState(StateMachine.State.OFF);
        fsm.update();
        drivetrain.periodic();
        shooter.periodic();
        intake.periodic();
        transfer.periodic();
        dataLog();
    }
}

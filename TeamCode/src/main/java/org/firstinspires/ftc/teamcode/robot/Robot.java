package org.firstinspires.ftc.teamcode.robot;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static org.firstinspires.ftc.teamcode.util.Constants.Gamepad.LEFT_TRIGGER_THRESHOLD;
import static org.firstinspires.ftc.teamcode.util.Constants.Gamepad.RIGHT_TRIGGER_THRESHOLD;

import com.pedropathing.ivy.Command;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.FlowerBar;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Outtake;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;
import org.firstinspires.ftc.teamcode.mechanisms.Transfer;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.PanelsPoseTelemetry;

import dev.nextftc.robot.NextRobot;
import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextOpMode;

public class Robot implements NextRobot {

    public enum State {
        IDLE,
        INTAKE,
        REVERSE,
        SHOOT_POLLEN,
        INTAKE_AND_SHOOT,
        INTAKE_AND_SLOW,
        HOLD_AND_SHOOT,
        FLOWER_BAR
    }

    private Gamepad gp1;
    private boolean autonomous;
    public Constants.Alliance alliance;

    private State state = State.IDLE;
    private State lastAppliedState = null;
    public Drivetrain drivetrain;
    public Intake intake;
    public Outtake shooter;
    public Transfer transfer;
    public FlowerBar flowerBar;
    public Turret turret;

    public Robot() {}

    public State getState() {
        return state;
    }

    public Command setState(State newState) {
        return instant(() -> state = newState);
    }
    private void configureKeybinds() {
        boolean leftTrigger = gp1.left_trigger > LEFT_TRIGGER_THRESHOLD;
        boolean rightTrigger = gp1.right_trigger > RIGHT_TRIGGER_THRESHOLD;
        boolean leftBumper = gp1.left_bumper;
        boolean rightBumper = gp1.right_bumper;
        boolean a = gp1.a;

        if (gp1.y) {
            state = State.REVERSE;
        } else if (leftTrigger && rightTrigger) {
            state = State.INTAKE_AND_SLOW;
        } else if (leftBumper && rightBumper) {
            state = State.INTAKE_AND_SHOOT;
        } else if (rightTrigger) {
            state = State.HOLD_AND_SHOOT;
        } else if (leftTrigger) {
            state = State.FLOWER_BAR;
        } else if (leftBumper) {
            state = State.INTAKE;
        } else if (rightBumper) {
            state = State.SHOOT_POLLEN;
        } else {
            state = State.IDLE;
        }

        if (a) {
            drivetrain.setPose(Constants.Drivetrain.RESET_POSE);
        }
    }

    private void applyState() {
        if (state == lastAppliedState) {
            return;
        }
        lastAppliedState = state;

        Intake.IntakeState intakeState = Intake.IntakeState.IDLE;
        Outtake.OuttakeState outtakeState = Outtake.OuttakeState.IDLE;
        Transfer.TransferState transferState = Transfer.TransferState.IDLE;
        FlowerBar.FlowerBarState flowerBarState = FlowerBar.FlowerBarState.STOW;
        Drivetrain.DrivetrainState drivetrainState = Drivetrain.DrivetrainState.MANUAL;

        switch (state) {
            case IDLE:
                break;

            case INTAKE:
                intakeState = Intake.IntakeState.RUNNING;
                transferState = Transfer.TransferState.INTAKE;
                break;

            case REVERSE:
                intakeState = Intake.IntakeState.REVERSE;
                transferState = Transfer.TransferState.REVERSE;
                break;

            case SHOOT_POLLEN:
                outtakeState = Outtake.OuttakeState.START_POLLEN;
                transferState = Transfer.TransferState.SHOOT;
                break;

            case INTAKE_AND_SHOOT:
                intakeState = Intake.IntakeState.RUNNING;
                outtakeState = Outtake.OuttakeState.START_POLLEN;
                transferState = Transfer.TransferState.SHOOT;
                break;

            case INTAKE_AND_SLOW:
                intakeState = Intake.IntakeState.RUNNING;
                transferState = Transfer.TransferState.INTAKE;
                drivetrainState = Drivetrain.DrivetrainState.SLOW;
                break;

            case HOLD_AND_SHOOT:
                drivetrainState = Drivetrain.DrivetrainState.HOLD;
                outtakeState = Outtake.OuttakeState.START_POLLEN;
                transferState = Transfer.TransferState.SHOOT;
                break;

            case FLOWER_BAR:
                flowerBarState = FlowerBar.FlowerBarState.DEPLOY;
                intakeState = Intake.IntakeState.RUNNING;
                transferState = Transfer.TransferState.INTAKE;
                break;
        }

        Command mechanismCommands = parallel(
                intake.setState(intakeState),
                shooter.setState(outtakeState),
                transfer.setState(transferState),
                flowerBar.setState(flowerBarState)
        );

        if (state == State.IDLE) {
            mechanismCommands = parallel(
                    mechanismCommands,
                    turret.setState(Turret.TurretState.TRACKING)
            );
        }
        if (!autonomous) {
            mechanismCommands = parallel(
                    mechanismCommands,
                    drivetrain.setState(drivetrainState)
            );
        }

        Scheduler.schedule(mechanismCommands);
    }

    public void dataLog() {
        Telemetry.log("Alliance: ", alliance);
        Telemetry.log("Robot State: ", state);

        Telemetry.log("Intake State: ", intake.getState());
        Telemetry.log("Outtake State: ", shooter.getState());
        Telemetry.log("Shooter RPM: ", shooter.getRPM());
        Telemetry.log("Shooter Ready: ", shooter.isReady());
        Telemetry.log("Transfer State: ", transfer.getState());
        Telemetry.log("Transfer Feeding: ", transfer.isFeeding());
        Telemetry.log("Flower Bar State: ", flowerBar.getState());
        Telemetry.log("Turret State: ", turret.getState());

        PanelsPoseTelemetry.update(drivetrain.getPose());
    }

    public void init(NextOpMode opMode, boolean autonomous) {
        this.autonomous = autonomous;
        this.gp1 = opMode.gamepad1;

        drivetrain = new Drivetrain(opMode.hardwareMap, opMode.gamepad1, alliance);
        turret = new Turret(this);
        intake = new Intake();
        shooter = new Outtake();
        transfer = new Transfer(shooter);
        flowerBar = new FlowerBar();

        state = State.IDLE;
        lastAppliedState = null;
    }

    @Override
    public void periodic() {
        if (!autonomous) {
            configureKeybinds();
        }

        applyState();

        drivetrain.periodic();
        intake.periodic();
        shooter.periodic();
        transfer.periodic();
        flowerBar.periodic();
        turret.periodic();

        dataLog();
    }
}

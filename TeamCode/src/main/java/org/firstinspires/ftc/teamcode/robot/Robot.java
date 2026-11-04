package org.firstinspires.ftc.teamcode.robot;

import static com.pedropathing.ivy.groups.Groups.parallel;
import static org.firstinspires.ftc.teamcode.util.Constants.Gamepad.leftTriggerThreshold;
import static org.firstinspires.ftc.teamcode.util.Constants.Gamepad.rightTriggerThreshold;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.FlowerBar;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Transfer;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.PanelsPoseTelemetry;

import dev.nextftc.robot.NextRobot;
import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextOpMode;

public class Robot implements NextRobot {

    public enum State {
        IDLE,
        INTAKE,
        SHOOT_POLLEN,
        INTAKE_AND_SHOOT,
        INTAKE_AND_SLOW,
        HOLD,
        FLOWER_BAR
    }

    private Gamepad gp1;
    private boolean autonomous;
    public Constants.Alliance alliance;

    private State state = State.IDLE;
    private State lastAppliedState = null;

    public Drivetrain drivetrain;
    public Intake intake;
    public Shooter shooter;
    public Transfer transfer;
    public FlowerBar flowerBar;
    public Turret turret;

    public Robot() {}

    public State getState() {
        return state;
    }

    public void setState(State newState) {
        if (newState != null) {
            state = newState;
        }
    }
    private void configureKeybinds() {
        boolean leftTrigger = gp1.left_trigger > leftTriggerThreshold;
        boolean rightTrigger = gp1.right_trigger > rightTriggerThreshold;
        boolean leftBumper = gp1.left_bumper;
        boolean rightBumper = gp1.right_bumper;

        if (leftTrigger) {
            state = State.FLOWER_BAR;
        } else if (leftBumper && rightBumper && rightTrigger) {
            state = State.INTAKE_AND_SHOOT;
        } else if (rightTrigger) {
            state = State.HOLD;
        } else if (leftBumper && rightBumper) {
            state = State.INTAKE_AND_SLOW;
        } else if (leftBumper) {
            state = State.INTAKE;
        } else if (rightBumper) {
            state = State.SHOOT_POLLEN;
        } else {
            state = State.IDLE;
        }
    }

    private void applyState() {
        if (state == lastAppliedState) {
            return;
        }
        lastAppliedState = state;

        Intake.IntakeState intakeState = Intake.IntakeState.IDLE;
        Shooter.ShooterState shooterState = Shooter.ShooterState.IDLE;
        Transfer.TransferState transferState = Transfer.TransferState.IDLE;
        FlowerBar.FlowerBarState flowerBarState = FlowerBar.FlowerBarState.STOW;
        Drivetrain.DrivetrainState drivetrainState = Drivetrain.DrivetrainState.MANUAL;

        switch (state) {
            case IDLE:
                break;

            case INTAKE:
                intakeState = Intake.IntakeState.RUNNING;
                transferState = Transfer.TransferState.RUNNING_INTAKE;
                break;

            case SHOOT_POLLEN:
                shooterState = Shooter.ShooterState.START_POLLEN;
                transferState = Transfer.TransferState.RUNNING_OUTTAKE;
                break;

            case INTAKE_AND_SHOOT:
                intakeState = Intake.IntakeState.RUNNING;
                shooterState = Shooter.ShooterState.START_POLLEN;
                transferState = Transfer.TransferState.RUNNING_BOTH;
                break;

            case INTAKE_AND_SLOW:
                intakeState = Intake.IntakeState.RUNNING;
                transferState = Transfer.TransferState.RUNNING_INTAKE;
                drivetrainState = Drivetrain.DrivetrainState.SLOW;
                break;

            case HOLD:
                drivetrainState = Drivetrain.DrivetrainState.HOLD;
                break;

            case FLOWER_BAR:
                flowerBarState = FlowerBar.FlowerBarState.DEPLOY;
                intakeState = Intake.IntakeState.RUNNING;
                transferState = Transfer.TransferState.RUNNING_INTAKE;
                break;
        }

        Command mechanismCommands = parallel(
                intake.setState(intakeState),
                shooter.setState(shooterState),
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
        Telemetry.log("Robot State: ", state);
        Telemetry.log("Drivetrain State: ", drivetrain.getState());
        PanelsPoseTelemetry.update(drivetrain.getPose());
    }

    public void init(NextOpMode opMode, boolean autonomous) {
        this.autonomous = autonomous;
        this.gp1 = opMode.gamepad1;

        drivetrain = new Drivetrain(opMode.hardwareMap, opMode.gamepad1);
        turret = new Turret(this);
        intake = new Intake();
        shooter = new Shooter();
        transfer = new Transfer();
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

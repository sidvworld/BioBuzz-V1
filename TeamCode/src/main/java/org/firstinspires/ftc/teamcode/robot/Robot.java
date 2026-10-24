package org.firstinspires.ftc.teamcode.robot;

import static com.pedropathing.ivy.groups.Groups.parallel;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;
import org.firstinspires.ftc.teamcode.util.Constants;

import dev.nextftc.robot.NextRobot;
import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.triggers.CommandGamepad;
public class Robot implements NextRobot {
    public CommandGamepad gp1;
    private boolean autonomous;
    public Constants.Alliance alliance;
    public StateMachine fsm = new StateMachine();
    public Drivetrain drivetrain;
    public Intake intake;
    public Turret turret;
    public Robot() {}

    private Command idleMechanisms() {
        return parallel(
                intake.setState(Intake.IntakeState.IDLE),
                turret.setState(Turret.TurretState.TRACKING)
        );
    }

    private void configureFSM() {
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
    }

    private void configureKeybinds() {

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
    }

    public void init(NextOpMode opMode, boolean autonomous){
        this.autonomous = autonomous;

        gp1 = new CommandGamepad(opMode.gamepad1);

        drivetrain = new Drivetrain(opMode.hardwareMap, opMode.gamepad1);
        intake = new Intake();
        turret = new Turret(this);

        configureFSM();
        if (!autonomous) configureKeybinds();
    }

    public void periodic(){
        fsm.update();
        drivetrain.periodic();
        intake.periodic();
        dataLog();
    }
}

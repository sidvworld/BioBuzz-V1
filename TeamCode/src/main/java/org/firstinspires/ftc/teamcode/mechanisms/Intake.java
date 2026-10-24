package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.util.Constants;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Intake implements Mechanism {
    NextMotor intakeMotor = Constants.Intake.intakeMotor;

    public enum IntakeState {
        OFF,
        IDLE,
        REVERSE,
        RUNNING
    }

    public IntakeState intakeState;

    public Intake() {
        intakeState = IntakeState.IDLE;
        intakeMotor.setDirection(NextMotor.Direction.REVERSE);
    }

    public void cycle() {
        switch (intakeState) {
            case OFF:
                intakeMotor.setThrottle(0);
                break;

            case IDLE:
                intakeMotor.setThrottle(Constants.Intake.intakeIdlePower);
                break;

            case REVERSE:
                intakeMotor.setThrottle(-Constants.Intake.intakeRunningPower);
                break;

            case RUNNING:
                intakeMotor.setThrottle(Constants.Intake.intakeRunningPower);
                break;

            default:
                setState(IntakeState.IDLE);
                break;
        }
    }

    public Command setState(IntakeState state) {
        return instant(() -> intakeState = state);
    }

    @Override
    public void periodic() {
        cycle();
    }
}
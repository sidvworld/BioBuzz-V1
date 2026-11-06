package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.util.Constants.Intake.*;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.util.Constants;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Intake implements Mechanism {
    NextMotor intakeMotor = Constants.Intake.intakeMotor;
    NextMotor transferMotor1 = Constants.Intake.transferMotor1;

    public enum IntakeState {
        OFF,
        IDLE,
        REVERSE,
        RUNNING
    }

    private IntakeState intakeState;

    public Intake() {
        intakeState = IntakeState.IDLE;

        intakeMotor.setDirection(INTAKE_MOTOR_DIRECTION);
        transferMotor1.setDirection(TRANSFER_MOTOR1_DIRECTION);
    }

    public void cycle() {
        switch (intakeState) {
            case OFF:
                intakeMotor.setThrottle(0);
                transferMotor1.setThrottle(0);
                break;

            case IDLE:
                intakeMotor.setThrottle(INTAKE_IDLE_POWER);
                transferMotor1.setThrottle(INTAKE_IDLE_POWER);
                break;

            case REVERSE:
                intakeMotor.setThrottle(-INTAKE_RUNNING_POWER);
                transferMotor1.setThrottle(-INTAKE_RUNNING_POWER);
                break;

            case RUNNING:
                intakeMotor.setThrottle(INTAKE_RUNNING_POWER);
                transferMotor1.setThrottle(INTAKE_RUNNING_POWER);
                break;

        }
    }

    public IntakeState getState() {
        return intakeState;
    }

    public Command setState(IntakeState state) {
        return instant(() -> intakeState = state);
    }

    @Override
    public void periodic() {
        cycle();
    }
}
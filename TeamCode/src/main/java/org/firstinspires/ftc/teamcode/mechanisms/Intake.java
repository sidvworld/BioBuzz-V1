package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.data_util.Devices;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Intake implements Mechanism {

    NextMotor intakeMotor = Devices.intakeMotor;

    public static final double POWER = 1.0;
    public static final double IDLE_POWER = 0.0;

    public IntakeState intakeState;

    public enum IntakeState {
        OFF,
        IDLE,
        RUNNING
    }

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
                intakeMotor.setThrottle(IDLE_POWER);
                break;

            case RUNNING:
                intakeMotor.setThrottle(POWER);
                break;

            default:
                setState(IntakeState.IDLE);
                break;
        }
    }

    public Command setState(IntakeState state) {
        return instant(() -> intakeState = state);
    }

    public Command toggleDirection() {
        return instant(() -> {
            NextMotor.Direction currentDirection = intakeMotor.getDirection();
            intakeMotor.setDirection(currentDirection == NextMotor.Direction.FORWARD ? NextMotor.Direction.REVERSE : NextMotor.Direction.FORWARD
            );
        });
    }

    @Override
    public void periodic() {
        cycle();
    }
}
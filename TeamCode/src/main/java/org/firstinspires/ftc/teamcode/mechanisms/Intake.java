package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.data_util.Devices;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.measuretypes.AngularVelocity;

public class Intake implements Mechanism {
    NextMotor intakeMotor = Devices.intakeMotor;
    public IntakeState intakeState;
    public enum IntakeState {
        OFF,
        RUNNING,
        IDLE
    }
    private final double POWER = 1.0;
    private final double IDLE_POWER = 0.0;
    public Intake(){
        intakeState = IntakeState.IDLE;
        intakeMotor.setDirection(NextMotor.Direction.REVERSE);
    }

    public Command run() {
        return instant(() -> {
            intakeState = IntakeState.RUNNING;
            intakeMotor.setThrottle(POWER);
        });
    }

    public Command toggleDirection(){
        NextMotor.Direction currentDirection = intakeMotor.getDirection();
        return instant(() -> intakeMotor.setDirection(currentDirection.equals(NextMotor.Direction.FORWARD) ? NextMotor.Direction.REVERSE : NextMotor.Direction.FORWARD));
    }

    public Command idle() {
        return instant(() -> {
            intakeState = IntakeState.IDLE;
            intakeMotor.setThrottle(IDLE_POWER);
        });
    }

    public double getSpeed(){
        return intakeMotor.getThrottle();
    }
}
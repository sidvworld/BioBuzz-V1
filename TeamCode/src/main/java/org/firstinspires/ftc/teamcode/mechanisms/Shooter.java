package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.util.Constants.Shooter.*;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.util.Constants;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Units;


public class Shooter implements Mechanism {
    NextMotor shooterMotor = Constants.Shooter.shooterMotor;
    public enum ShooterState{
        OFF,
        IDLE,
        REVERSE,
        START_POLLEN,
    }

    public ShooterState shooterState;
    public boolean IS_READY;
    private long readySinceNs;

    public Shooter() {
        shooterMotor.setDirection(NextMotor.Direction.REVERSE);
        shooterMotor.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.getVelocityConstants().withP(KP).withV(KV);

        shooterState = ShooterState.IDLE;
        IS_READY = false;
    }

    public void cycle() {
        switch (shooterState) {
            case OFF:
                shooterMotor.setThrottle(0);
                break;
            case IDLE:
                shooterMotor.setVelocitySetpoint(Units.getRotationsPerMinute(IDLE_SETPOINT));
                break;
            case REVERSE:
                shooterMotor.setVelocitySetpoint(Units.getRotationsPerMinute(-IDLE_SETPOINT));
                break;
            case START_POLLEN:
                shooterMotor.setVelocitySetpoint(Units.getRotationsPerMinute(SHOOTING_SETPOINT));
                break;
            default:
                shooterState = ShooterState.IDLE;
                break;
        }
    }

    public Command setState(ShooterState state) {
        return instant(() -> {
            IS_READY = false;
            readySinceNs = 0;
            shooterState = state;
        });
    }

    public double getRPM(){
        return shooterMotor.getEncoderVelocity().into(Units.RotationsPerMinute);
    }

    public double getRPMSetpoint() {
        switch (shooterState) {
            case IDLE: return IDLE_SETPOINT;
            case REVERSE: return -IDLE_SETPOINT;
            case START_POLLEN: return SHOOTING_SETPOINT;
            default: return 0;
        }
    }

    public void checkThreshold(){
        if (shooterState == ShooterState.START_POLLEN && Math.abs(getRPM() - SHOOTING_SETPOINT) <= SHOOTING_THRESHOLD) {
            long now = System.nanoTime();
            if (readySinceNs == 0) readySinceNs = now;
            IS_READY = now - readySinceNs >= READY_TIME_MS * 1_000_000L;
        } else {
            readySinceNs = 0;
            IS_READY = false;
        }
    }

    public boolean isReady(){
        return IS_READY;
    }


    @Override
    public void periodic() {
        cycle();
        checkThreshold();
    }
}

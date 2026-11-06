package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.util.Constants.Outtake.*;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.util.Constants;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Units;

public class Outtake implements Mechanism {
    NextMotor shooterMotor = Constants.Outtake.shooterMotor;
    public enum OuttakeState {
        OFF,
        IDLE,
        START_POLLEN
    }

    private OuttakeState outtakeState;
    private boolean ready;
    private long readySinceNs;

    public Outtake() {
        outtakeState = OuttakeState.IDLE;
        ready = false;

        shooterMotor.setDirection(SHOOTER_MOTOR_DIRECTION);
        shooterMotor.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        shooterMotor.getVelocityConstants().withP(KP).withV(KV);
    }

    public void cycle() {
        switch (outtakeState) {
            case OFF:
                shooterMotor.setThrottle(0);
                break;

            case IDLE:
                shooterMotor.setVelocitySetpoint(Units.getRotationsPerMinute(IDLE_SETPOINT));
                break;

            case START_POLLEN:
                shooterMotor.setVelocitySetpoint(Units.getRotationsPerMinute(SHOOTING_SETPOINT));
                break;

        }
    }

    public OuttakeState getState() {
        return outtakeState;
    }

    public Command setState(OuttakeState state) {
        return instant(() -> {
            if (outtakeState == state) return;
            ready = false;
            readySinceNs = 0;
            outtakeState = state;
        });
    }

    public double getRPM(){
        return shooterMotor.getEncoderVelocity().into(Units.RotationsPerMinute);
    }

    public double getRPMSetpoint() {
        switch (outtakeState) {
            case IDLE: return IDLE_SETPOINT;
            case START_POLLEN: return SHOOTING_SETPOINT;
            default: return 0;
        }
    }

    public void checkThreshold(){
        if (outtakeState == OuttakeState.START_POLLEN && Math.abs(getRPM() - SHOOTING_SETPOINT) <= SHOOTING_THRESHOLD) {
            long now = System.nanoTime();
            if (readySinceNs == 0) readySinceNs = now;
            ready = now - readySinceNs >= READY_TIME_MS * 1_000_000L;
        } else {
            readySinceNs = 0;
            ready = false;
        }
    }

    public boolean isReady(){
        return ready;
    }

    @Override
    public void periodic() {
        cycle();
        checkThreshold();
    }
}

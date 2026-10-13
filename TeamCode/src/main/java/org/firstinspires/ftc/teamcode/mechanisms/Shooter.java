package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.util.Devices;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Units;


public class Shooter implements Mechanism {
    NextMotor shooterMotor = Devices.shooterMotor;
    // NextServo shooterServo = Devices.shooterServo;

    public enum ShooterState{
        OFF,
        IDLE,
        REVERSE,
        START_POLLEN,
    }

    public static final double SHOOTING_SETPOINT = 3000;
    public static final double SHOOTING_THRESHOLD = 300;
    public static final long READY_TIME_MS = 100;
    public static final double IDLE_SETPOINT = SHOOTING_SETPOINT * 0.5;
    // public static final double SERVO_POSE_POLLEN = 0.45;

    public ShooterState shooterState;
    public boolean IS_READY;
    private long readySinceNanos;

    public Shooter() {
        shooterMotor.setDirection(NextMotor.Direction.REVERSE);
        shooterMotor.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.FLOAT);
        // starting gains for the 6000 RPM motor
        shooterMotor.getVelocityConstants().withP(0.0002).withV(1.0 / 6000.0);

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
                // shooterServo.setPosition(SERVO_POSE_POLLEN);
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
            readySinceNanos = 0;
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
        if (shooterState == ShooterState.START_POLLEN
                && Math.abs(getRPM() - SHOOTING_SETPOINT) <= SHOOTING_THRESHOLD) {
            long now = System.nanoTime();
            if (readySinceNanos == 0) readySinceNanos = now;
            // Require continuous time at speed before starting or resuming feeding.
            IS_READY = now - readySinceNanos >= READY_TIME_MS * 1_000_000L;
        } else {
            readySinceNanos = 0;
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

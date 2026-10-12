package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.data_util.Devices;

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
    public static final double IDLE_SETPOINT = SHOOTING_SETPOINT * 0.5;
    // public static final double SERVO_POSE_POLLEN = 0.45;

    public ShooterState shooterState;
    public boolean IS_READY;

    public Shooter() {
        shooterMotor.setDirection(NextMotor.Direction.REVERSE);
        shooterMotor.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.FLOAT);

        shooterState = ShooterState.IDLE;
        IS_READY = false;
    }

    public void cycle() {
        switch (shooterState) {
            case OFF:
                shooterMotor.setThrottle(0);
                break;
            case IDLE:
                shooterMotor.setThrottle(IDLE_SETPOINT);
                break;
            case REVERSE:
                shooterMotor.setThrottle(-IDLE_SETPOINT);
                break;
            case START_POLLEN:
                // shooterServo.setPosition(SERVO_POSE_POLLEN);
                shooterMotor.setVelocitySetpoint(Units.getRotationsPerMinute(SHOOTING_SETPOINT));
                break;
            default:
                setState(ShooterState.IDLE);
                break;
        }
    }

    public Command setState(ShooterState state) {
        return instant(() -> shooterState = state);
    }

    public double getRPM(){
        return shooterMotor.getEncoderVelocity().getMagnitude();
    }

    public void checkThreshold(){
        IS_READY = Math.abs(getRPM()) >= SHOOTING_SETPOINT - SHOOTING_THRESHOLD;
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

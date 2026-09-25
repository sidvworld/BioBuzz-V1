package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.data_util.Devices;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;

public class Shooter implements Mechanism {

    NextMotor shooterMotor = Devices.shooterMotor;
    NextServo shooterServo = Devices.shooterServo;

    public static final double POWER = 0.5;
    public static final double SHOOTING_THRESHOLD = 2500;
    public static final double IDLE_POWER = 0.25;
    public static final double SERVO_POSE_NECTAR = 0.85;
    public static final double SERV_POSE_POLLEN = 0.45;

    public ShooterState shooterState;
    public boolean IS_READY;

    public enum ShooterState{
        OFF,
        IDLE,
        START_POLLEN,
        START_NECTAR,
    }

    public Shooter() {
        shooterMotor.setDirection(NextMotor.Direction.REVERSE);
        shooterState = ShooterState.IDLE;
        IS_READY = false;
    }

    public Command run(){
        if (shooterState == ShooterState.START_NECTAR){
            return instant(() -> {
                shooterMotor.setThrottle(POWER);
                shooterServo.setPosition(SERVO_POSE_NECTAR);
            });
        } else if (shooterState == ShooterState.START_POLLEN){
            return instant(() -> {
                shooterMotor.setThrottle(POWER);
                shooterServo.setPosition(SERV_POSE_POLLEN);
            });
        } else {
            return idle();
        }
    }

    public Command idle(){
        return instant(() -> {
            shooterState = ShooterState.IDLE;
            shooterMotor.setThrottle(IDLE_POWER);
        });
    }

    public Command off(){
        return instant(() -> {
            shooterState = ShooterState.OFF;
            shooterMotor.setThrottle(0);
        });
    }

    public void checkThreshold(){
        IS_READY = Math.abs(shooterMotor.getEncoderVelocity().getMagnitude()) >= SHOOTING_THRESHOLD;
    }

    public boolean isReady(){
        return IS_READY;
    }

    public Command toggleBall(){
        return instant(() -> { shooterState = shooterState.equals(ShooterState.START_POLLEN) ? ShooterState.START_NECTAR : ShooterState.START_POLLEN; });
    }

    @Override
    public void periodic() {
        checkThreshold();
    }
}

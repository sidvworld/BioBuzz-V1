package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.data_util.Devices;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Transfer implements Mechanism {
    NextMotor transferMotor = Devices.transferMotor;
    public static final double POWER = 1.0;
    public static final double IDLE_POWER = 0.0;

    public TransferState transferState;
    public enum TransferState {
        OFF,
        IDLE,
        RUNNING
    }

    public Transfer(){
        transferState = TransferState.IDLE;
        transferMotor.setDirection(NextMotor.Direction.FORWARD);
    }

    public Command run(){
        return instant(() -> {
            transferState = TransferState.RUNNING;
            transferMotor.setThrottle(POWER);
        });
    }

    public Command idle() {
        return instant(() -> {
            transferState = TransferState.IDLE;
            transferMotor.setThrottle(IDLE_POWER);
        });
    }

    public Command toggleDirection(){
        NextMotor.Direction currentDirection = transferMotor.getDirection();
        return instant(() -> transferMotor.setDirection(currentDirection.equals(NextMotor.Direction.FORWARD) ? NextMotor.Direction.REVERSE : NextMotor.Direction.FORWARD));
    }

}
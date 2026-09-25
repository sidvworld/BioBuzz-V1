package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.data_util.Devices;

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

    public Transfer() {
        transferState = TransferState.IDLE;
        transferMotor.setDirection(NextMotor.Direction.FORWARD);
    }

    public void cycle() {
        switch (transferState) {
            case OFF:
                transferMotor.setThrottle(0);
                break;

            case IDLE:
                transferMotor.setThrottle(IDLE_POWER);
                break;

            case RUNNING:
                transferMotor.setThrottle(POWER);
                break;

            default:
                setState(TransferState.IDLE);
                break;
        }
    }

    public Command setState(TransferState state) {
        return instant(() -> transferState = state);
    }

    public Command toggleDirection() {
        return instant(() -> {
            NextMotor.Direction currentDirection = transferMotor.getDirection();
            transferMotor.setDirection(currentDirection == NextMotor.Direction.FORWARD ? NextMotor.Direction.REVERSE : NextMotor.Direction.FORWARD
            );
        });
    }

    @Override
    public void periodic() {
        cycle();
    }
}
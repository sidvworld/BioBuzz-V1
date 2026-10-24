package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.util.Constants.Transfer.*;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.util.Constants;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Transfer implements Mechanism {
    NextMotor transferMotor = Constants.Transfer.transferMotor;
    public enum TransferState {
        OFF,
        IDLE,
        REVERSE,
        RUNNING
    }

    public TransferState transferState;
    public Transfer() {
        transferMotor.setDirection(NextMotor.Direction.FORWARD);
        transferState = TransferState.IDLE;
    }

    public void cycle() {
        switch (transferState) {
            case OFF:
                transferMotor.setThrottle(0);
                break;

            case IDLE:
                transferMotor.setThrottle(IDLE_POWER);
                break;

            case REVERSE:
                transferMotor.setThrottle(-POWER);
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

    @Override
    public void periodic() {
        cycle();
    }
}
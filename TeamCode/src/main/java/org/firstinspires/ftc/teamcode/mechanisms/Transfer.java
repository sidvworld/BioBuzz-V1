package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.util.Constants.Transfer.*;

import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Transfer implements Mechanism {
    NextMotor transferMotor = transferMotor2;
    private final Outtake shooter;

    public enum TransferState {
        IDLE,
        INTAKE,
        REVERSE,
        SHOOT
    }

    private TransferState transferState = TransferState.IDLE;
    private long reverseStartedNs;
    private boolean feeding;

    public Transfer(Outtake shooter) {
        this.shooter = shooter;
        transferMotor.setDirection(TRANSFER_MOTOR2_DIRECTION);
    }

    public void cycle() {
        feeding = false;
        switch (transferState) {
            case IDLE:
                transferMotor.setThrottle(IDLE_POWER);
                break;

            case INTAKE:
                transferMotor.setThrottle(TRANSFER_RUNNING_POWER / 2);
                break;

            case REVERSE:
                transferMotor.setThrottle(-TRANSFER_RUNNING_POWER);
                break;

            case SHOOT:
                if (System.nanoTime() - reverseStartedNs < REVERSE_TIME_MS * 1_000_000L) {
                    transferMotor.setThrottle(-TRANSFER_RUNNING_POWER / 2);
                } else {
                    feeding = shooter.isReady();
                    transferMotor.setThrottle(feeding ? TRANSFER_RUNNING_POWER : 0);
                }
                break;
        }
    }

    public TransferState getState() {
        return transferState;
    }

    public Command setState(TransferState state) {
        return instant(() -> {
            if (transferState == state) return;
            transferState = state;
            reverseStartedNs = System.nanoTime();
            feeding = false;
        });
    }

    public boolean isFeeding() {
        return feeding;
    }

    @Override
    public void periodic() {
        cycle();
    }
}

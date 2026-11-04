package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.util.Constants.Transfer.*;

import com.pedropathing.ivy.Command;
import org.firstinspires.ftc.teamcode.util.Constants;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Transfer implements Mechanism {

    NextMotor transferMotor1 = Constants.Transfer.transferMotor1;
    NextMotor transferMotor2 = Constants.Transfer.transferMotor2;

    public enum TransferState {
        OFF,
        IDLE,
        REVERSE,
        RUNNING_INTAKE,
        RUNNING_OUTTAKE,
        RUNNING_BOTH
    }

    public TransferState transferState = TransferState.IDLE;
    private boolean outtakeEnabled = false;

    public Transfer() {
        transferMotor1.setDirection(NextMotor.Direction.FORWARD);
        transferMotor2.setDirection(NextMotor.Direction.FORWARD);
    }

    public void setOuttakeEnabled(boolean enabled) {
        outtakeEnabled = enabled;
    }

    public void cycle() {
        switch (transferState) {
            case OFF:
                transferMotor1.setThrottle(0);
                transferMotor2.setThrottle(0);
                break;

            case IDLE:
                transferMotor1.setThrottle(IDLE_POWER);
                transferMotor2.setThrottle(0);
                break;

            case REVERSE:
                transferMotor1.setThrottle(-POWER);
                transferMotor2.setThrottle(-POWER);
                break;

            case RUNNING_INTAKE:
                transferMotor1.setThrottle(POWER);
                transferMotor2.setThrottle(0);
                break;

            case RUNNING_OUTTAKE:
                transferMotor1.setThrottle(0);
                transferMotor2.setThrottle(outtakeEnabled ? POWER : 0);
                break;

            case RUNNING_BOTH:
                transferMotor1.setThrottle(POWER);
                transferMotor2.setThrottle(outtakeEnabled ? POWER : 0);
                break;

            default:
                transferState = TransferState.IDLE;
                transferMotor1.setThrottle(IDLE_POWER);
                transferMotor2.setThrottle(0);
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

package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Utilities;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;

public class FlowerBar implements Mechanism {
    public NextServo flowerBarServo = Constants.FlowerBar.flowerBarServo;

    public enum FlowerBarState {
        STOW,
        DEPLOY
    }

    private FlowerBarState flowerBarState;

    public FlowerBar() {
        flowerBarState = FlowerBarState.STOW;
        flowerBarServo.setDirection(Constants.FlowerBar.flowerBarServoDirection);
    }

    public void cycle() {
        switch (flowerBarState) {
            case STOW   :
                flowerBarServo.setPosition(Constants.FlowerBar.STOW_POSE);
                break;

            case DEPLOY:
                flowerBarServo.setPosition(Constants.FlowerBar.DEPLOY_POSE);
                break;

            default:
                break;
        }
    }

    public FlowerBarState getState() {
        return flowerBarState;
    }

    public Command setState(FlowerBarState state) {
        return instant(() -> flowerBarState = state);
    }

    @Override
    public void periodic() {
        cycle();
    }
}
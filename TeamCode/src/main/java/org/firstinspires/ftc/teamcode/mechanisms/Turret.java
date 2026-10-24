package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.ShotCalculator;
import org.firstinspires.ftc.teamcode.util.Utilities;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.Telemetry;

public class Turret implements Mechanism {
    NextServo turretServo1 = Constants.Turret.turretServo1;
    NextServo turretServo2 = Constants.Turret.turretServo2;

    public enum TurretState {
        TRACKING,
        CENTER
    }

    private final Robot robot;

    public TurretState turretState = TurretState.TRACKING;

    private double angle = 0.0;
    private double distance = 0.0;

    public Turret(Robot robot) {
        this.robot = robot;

        turretServo1.setDirection(Constants.Turret.turretServoDirection1);
        turretServo2.setDirection(Constants.Turret.turretServoDirection2);
    }

    public static double toPosition(double angleRadians) {
        double max = Constants.Turret.rangeDegrees / 2.0;

        double degrees = Utilities.clamp(
                Math.toDegrees(angleRadians), -max, max
        );

        return 0.5 - degrees / Constants.Turret.rangeDegrees;
    }

    public void updateTurret(double angleRadians) {
        angle = angleRadians;

        double position = toPosition(angleRadians);

        turretServo1.setPosition(position);
        turretServo2.setPosition(position);
    }

    public void cycle() {
        switch (turretState) {

            case TRACKING:
                ShotCalculator.Solution shot = ShotCalculator.calculate(
                        robot.drivetrain.getPose(),
                        robot.alliance
                );

                distance = shot.distance;
                updateTurret(shot.turretAngle);
                break;

            case CENTER:
                distance = 0.0;
                updateTurret(0.0);
                break;
        }
    }

    public Command setState(TurretState state) {
        return instant(() -> turretState = state);
    }

    @Override
    public void periodic() {
        cycle();
        Telemetry.log("turret state", turretState);
        Telemetry.log("turret angle (deg)", Math.toDegrees(angle));
        Telemetry.log("turret position", toPosition(angle));
        Telemetry.log("goal distance", distance);
    }
}
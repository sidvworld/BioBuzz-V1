package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.units.Units;

public class Constants {
    public enum Alliance {
        BLUE,
        RED
    }
    public static class Gamepad {
        public static final double LEFT_TRIGGER_THRESHOLD = 0.2;
        public static final double RIGHT_TRIGGER_THRESHOLD = 0.2;
    }
    public static class Intake {
        public static final NextMotor intakeMotor = new NextMotor(RobotController.expansionHub(), 0);
        public static final NextMotor.Direction INTAKE_MOTOR_DIRECTION = NextMotor.Direction.REVERSE;

        public static final NextMotor transferMotor1 = new NextMotor(RobotController.controlHub(), 1);
        public static final NextMotor.Direction TRANSFER_MOTOR1_DIRECTION = NextMotor.Direction.REVERSE;

        public static final double INTAKE_IDLE_POWER = 0.0;
        public static final double INTAKE_RUNNING_POWER = 1.0;
    }

    public static class Outtake {
        // 6000 RPM goBILDA 5203 motor, 1:1 flywheel: 28 encoder counts per revolution.
        public static final NextMotor shooterMotor = new NextMotor(RobotController.controlHub(), 0, Units.getRadians(2.0 * Math.PI / 28.0));
        public static final NextMotor.Direction SHOOTER_MOTOR_DIRECTION = NextMotor.Direction.REVERSE;
        public static final NextMotor.ZeroPowerBehavior ZERO_POWER_BEHAVIOR = NextMotor.ZeroPowerBehavior.FLOAT;

        public static double SHOOTING_SETPOINT = 3000;
        public static final double SHOOTING_THRESHOLD = 300;
        public static final long READY_TIME_MS = 100;
        public static final double IDLE_SETPOINT = SHOOTING_SETPOINT * 0.5;

        // Starting gains in motor power per RPM; tune on the robot under load.
        public static final double KP = 0.02;
        public static final double KV = 1.0 / 6000.0;
        public static final int SHOOT_TIMEOUT_MS = 5000;
        public static final int FEED_TIME_MS = 200;
    }

    public static class Transfer {

        public static final NextMotor transferMotor2 = new NextMotor(RobotController.expansionHub(), 3);
        public static final NextMotor.Direction TRANSFER_MOTOR2_DIRECTION = NextMotor.Direction.FORWARD;
        public static final double TRANSFER_RUNNING_POWER = 1.0;
        public static final double IDLE_POWER = 0.0;
        public static final int REVERSE_TIME_MS = 100;
    }
    public static class Turret {
        public static final NextServo turretServo1 = new NextServo(RobotController.controlHub(), 1, Constants.Turret.servoCacheTolerance);
        public static final NextServo turretServo2 = new NextServo(RobotController.controlHub(), 2, Constants.Turret.servoCacheTolerance);

        public static final NextMotor.Direction turretServoDirection1 = NextMotor.Direction.FORWARD;
        public static final NextMotor.Direction turretServoDirection2 = NextMotor.Direction.FORWARD;
        public static final double servoCacheTolerance = 0.001;

        public static final double rangeDegrees = 355;
        public static final Pose turretOffset = new Pose(0, 0);
    }

    public static class FlowerBar {
        public static final NextServo flowerBarServo = new NextServo(RobotController.controlHub(), 0, Constants.FlowerBar.servoCacheTolerance);
        public static final NextMotor.Direction flowerBarServoDirection = NextMotor.Direction.FORWARD;

        public static final double STOW_POSE = 0;
        public static final double DEPLOY_POSE = 0.655;
        public static final double servoCacheTolerance = 0.001;
    }

    public static class ShotCalculator {
        public static final Pose blueGoalLow = new Pose(85.4, 55.5);
        public static final Pose blueGoalHigh = new Pose(85.4, 82.6);
        public static final Pose redGoalLow = new Pose(59.0, 60.8);
        public static final Pose redGoalHigh = new Pose(59.0, 87.9);
    }

    public static com.pedropathing.drivetrain.Drivetrain drivetrain(HardwareMap h) {
        return new Mecanum(h, Drivetrain.mecanumConfig);
    }

    public static Localizer localizer(HardwareMap h) {
        return new PinpointLocalizer(h, Drivetrain.pinpointConfig);
    }

    public static Algorithm algorithm() {
        return new Foresight(Drivetrain.foresightConfig);
    }

    public static Follower create(HardwareMap h) {
        return new Follower(localizer(h), drivetrain(h), algorithm());
    }

    public static class Drivetrain {
        public static final Pose START_POSE = new Pose(7, 7, Math.toRadians(90));
        public static final Pose RESET_POSE = new Pose(7, 7, Math.toRadians(90));

        public static final double DRIVE_SCALE = 1.0;
        public static final double SLOW_SCALE = 0.25;
        public static final double DEADBAND = 0.05;

        public static final MecanumConfig mecanumConfig = new MecanumConfig(c -> {
            c.frontLeftName.set("fl");
            c.frontRightName.set("fr");
            c.backLeftName.set("bl");
            c.backRightName.set("br");
            c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
            c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        });

        public static final PinpointConfig pinpointConfig = new PinpointConfig(c -> {
            c.name.set("pinpoint");
            c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            c.xPodOffset.set(3.489964702936608);
            c.yPodOffset.set(-4.930589931217705);
            c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            c.globalDistanceUnit.set(DistanceUnit.INCH);
            c.offsetUnits.set(DistanceUnit.INCH);
        });

        public static final ForesightConfig foresightConfig = new ForesightConfig(c -> {
            Controller primaryTranslationalForward = Controller.proportional(0.31695896638586846);
            Controller secondaryTranslationalForward = Controller.proportional(0.11710786878001524);
            Controller primaryTranslationalLateral = Controller.proportional(0.49270594302169024);
            Controller secondaryTranslationalLateral = Controller.proportional(0.1820416806012474);

            c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
            c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

            c.coast.set(Controller.proportionalFeedforward(0.012707253306560775));
            c.brake.set(Controller.proportionalFeedforward(0.010801165310576658));

            c.headingFeedback.set(Controller.proportional(6.100494962820854));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(0.050830117376764294, 0.008351748206249005));

            c.linearBrakeCoefficients.set(Matrix.diag(0.05668663619191991, 0.06990910975077083));
            c.quadraticBrakeCoefficients.set(Matrix.diag(0.001929587469976467, 0.0013264711238913005));

            c.maxAchievableForwardVelocity.set(76.20941420165023);
            c.maxAchievableStrafeVelocity.set(62.07150168356044);
            c.naturalForwardDeceleration.set(41.12426767940491);
            c.naturalStrafeDeceleration.set(64.63576436632204);
        });
    }
}

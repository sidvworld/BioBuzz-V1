package org.firstinspires.ftc.teamcode.data_util;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;

public class Devices {

    public static final String fl = "fl";
    public static final String fr = "fr";
    public static final String bl = "bl";
    public static final String br = "br";


    public static final NextMotor intakeMotor = new NextMotor(RobotController.controlHub(), 0);


    public static final NextMotor shooterMotor = new NextMotor(RobotController.controlHub(), 1);
    public static final NextServo shooterServo = new NextServo(RobotController.controlHub(), 0);



    public static final NextMotor transferMotor = new NextMotor(RobotController.expansionHub(), 0);

}

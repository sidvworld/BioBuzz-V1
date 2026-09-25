package org.firstinspires.ftc.teamcode.data_util;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;

public class Devices {

    public static NextMotor flDrive = new NextMotor(RobotController.controlHub(), 2);
    public static NextMotor blDrive = new NextMotor(RobotController.controlHub(), 3);
    public static NextMotor frDrive = new NextMotor(RobotController.expansionHub(), 2);
    public static NextMotor brDrive = new NextMotor(RobotController.expansionHub(), 3);



    public static NextMotor intakeMotor = new NextMotor(RobotController.controlHub(), 0);


    public static NextMotor shooterMotor = new NextMotor(RobotController.controlHub(), 1);
    public static NextServo shooterServo = new NextServo(RobotController.controlHub(), 0);



    public static NextMotor transferMotor = new NextMotor(RobotController.expansionHub(), 0);

}

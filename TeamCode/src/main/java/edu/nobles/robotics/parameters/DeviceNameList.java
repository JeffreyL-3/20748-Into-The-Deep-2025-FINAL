package edu.nobles.robotics.parameters;

import com.acmerobotics.dashboard.config.Config;

//Key: if not specified, is on control hub

@Config
public class DeviceNameList {

    // Motors
    public static String vertSlideUpName = "motorExp0";
    public static String vertSlideDownName = "motorExp1";
    public static String horizontalSlide = "motorExp2";

    // Horizontal slide extension = front
    public static String frontLeftName = "motor3";
    public static String frontRightName = "motor1";
    public static String backLeftName = "motor2";
    public static String backRightName = "motor0";

    // Servos
    public static String clawName = "servo2";
    public static String intake1spinName = "servoExp0";
    public static String intake1FlipName = "servo0";

    // Sensor
    public static String color = "color";
    public static String touch = "touch";

}

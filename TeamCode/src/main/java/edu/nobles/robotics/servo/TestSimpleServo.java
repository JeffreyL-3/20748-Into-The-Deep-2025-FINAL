package edu.nobles.robotics.servo;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config

@TeleOp
public class TestSimpleServo extends LinearOpMode {
    public static String servoName = "servo0";

    public static int slowdown = 2;
    public static int aTurnTo = 150;
    public static double turnBy = 10;
    public static int minAng = 0;
    public static int maxAng = 355;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);;
        SimpleServo flip0 = new SimpleServo(
                hardwareMap, servoName, minAng, maxAng,
                AngleUnit.DEGREES
        );
        FtcDashboard dashboard = FtcDashboard.getInstance();


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            gamepadEx1.readButtons();

            double triggerSpin = gamepad1.left_stick_y;
            //flip0.rotateByAngle(triggerSpin/slowdown, AngleUnit.DEGREES);

            if(gamepadEx1.wasJustPressed(GamepadKeys.Button.X)) {
                flip0.rotateByAngle(turnBy, AngleUnit.DEGREES);
            }
            if(gamepadEx1.wasJustPressed(GamepadKeys.Button.Y)) {
                flip0.rotateByAngle(-turnBy, AngleUnit.DEGREES);
            }
            if(gamepad1.a){
                flip0.turnToAngle(aTurnTo, AngleUnit.DEGREES);
            }
            else if (gamepad1.b) {
                flip0.setPosition(0);
            }

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Angle", flip0.getAngle());
            telemetry.addData("Angle", flip0.getAngle());

            packet.put("status", "alive");
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();


        }
    }
}
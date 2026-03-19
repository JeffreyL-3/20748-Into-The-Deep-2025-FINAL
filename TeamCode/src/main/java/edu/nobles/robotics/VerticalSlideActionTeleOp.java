package edu.nobles.robotics;

import static org.firstinspires.ftc.teamcode.ANewActionTeleOpMode.claw1_closeDegree;
import static org.firstinspires.ftc.teamcode.ANewActionTeleOpMode.claw1_openDegree;
import static edu.nobles.robotics.parameters.DeviceNameList.clawName;
import static edu.nobles.robotics.parameters.DeviceNameList.vertSlideDownName;
import static edu.nobles.robotics.parameters.DeviceNameList.vertSlideUpName;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ANewActionTeleOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

import edu.nobles.robotics.motor.SlideActionMotor;
import edu.nobles.robotics.parameters.DeviceNameList;
import edu.nobles.robotics.servo.ServoDevice;


@TeleOp
@Config
public class VerticalSlideActionTeleOp extends LinearOpMode {
    public static int up_position = 7000;
    public static int up_score_position = 4700;


    private List<Action> runningActions = new ArrayList<>();

    MotorEx slideMotor;
    boolean manualMode;
    private ServoDevice clawServo;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slideMotor = new MotorEx(hardwareMap, vertSlideUpName, Motor.GoBILDA.RPM_435);
        slideMotor.stopAndResetEncoder();
        MotorEx vertSlideDown = new MotorEx(hardwareMap, vertSlideDownName, Motor.GoBILDA.RPM_312);
        clawServo = new ServoDevice(clawName, hardwareMap, telemetry, 355);

        SlideActionMotor motorSlide0 = new SlideActionMotor(slideMotor, vertSlideDown, telemetry, "simpleMotor", -1);
        motorSlide0.setManualMode();

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);

        waitForStart();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            gamepadEx1.readButtons();
            gamepadEx2.readButtons();

            if (manualMode) {
                double power = Range.clip(-gamepad2.left_stick_y, -ANewActionTeleOpMode.vertSlide_maxPower_up, ANewActionTeleOpMode.vertSlide_maxPower_up);
                motorSlide0.slideUpMotor.set(power);
            }

            if (gamepadEx2.wasJustPressed(GamepadKeys.Button.START)) {
                manualMode = !manualMode;
                if (manualMode)
                    motorSlide0.setManualMode();
                else
                    motorSlide0.setActionMode();
            }

            if (gamepadEx2.wasJustPressed(GamepadKeys.Button.Y)) {
                addActionEx(motorSlide0.moveSlide(up_position));
            }

            if (gamepadEx2.wasJustPressed(GamepadKeys.Button.A)) {
                addActionEx(motorSlide0.moveSlide(up_score_position));
            }

            if (gamepadEx2.wasJustPressed(GamepadKeys.Button.BACK)) {
                addActionEx(motorSlide0.moveSlide(0));
            }

            if (gamepadEx2.wasJustPressed(GamepadKeys.Button.BACK)) {
                addActionEx(motorSlide0.moveSlide(0));
            }

            if (gamepadEx2.wasJustPressed(GamepadKeys.Button.X)) {
                if (clawServo.available)
                    clawServo.servo.turnToAngle(claw1_openDegree, AngleUnit.DEGREES);
            }

            if (gamepadEx2.wasJustPressed(GamepadKeys.Button.B)) {
                if (clawServo.available)
                    clawServo.servo.turnToAngle(claw1_closeDegree, AngleUnit.DEGREES);
            }


            // update running actions
            if (!runningActions.isEmpty()) {
                List<Action> newActions = new ArrayList<>();
                for (Action action : runningActions) {
                    action.preview(packet.fieldOverlay());
                    if (action.run(packet)) {
                        newActions.add(action);
                    }
                }
                runningActions = newActions;
            }

            report(drive, packet);
        }
    }

    private void report(MecanumDrive drive, TelemetryPacket packet) {
        telemetry.addData("vertSlide position", slideMotor.getCurrentPosition());
        telemetry.addData("Manual Mode", manualMode);
        telemetry.update();
    }

    private void addActionEx(ActionEx actionEx) {
        removeExistingAction(actionEx.getDeviceName());
        runningActions.add(actionEx);
    }

    private void removeExistingAction(String deviceName) {
        // Remove current action
        runningActions.removeIf(a -> a instanceof ActionEx && deviceName.equals(((ActionEx) a).getDeviceName()));
    }
}

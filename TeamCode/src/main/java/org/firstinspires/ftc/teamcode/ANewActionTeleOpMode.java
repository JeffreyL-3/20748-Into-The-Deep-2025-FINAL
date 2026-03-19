package org.firstinspires.ftc.teamcode;

import static edu.nobles.robotics.parameters.DeviceNameList.clawName;
import static edu.nobles.robotics.parameters.DeviceNameList.horizontalSlide;
import static edu.nobles.robotics.parameters.DeviceNameList.intake1FlipName;
import static edu.nobles.robotics.parameters.DeviceNameList.intake1spinName;
import static edu.nobles.robotics.parameters.DeviceNameList.vertSlideDownName;
import static edu.nobles.robotics.parameters.DeviceNameList.vertSlideUpName;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import edu.nobles.robotics.ActionEx;
import edu.nobles.robotics.motor.HorizontalExtender;
import edu.nobles.robotics.motor.MotorGroupEx;
import edu.nobles.robotics.motor.SlideManualMotor;
import edu.nobles.robotics.servo.ServoDevice;

@Config
@TeleOp
public class ANewActionTeleOpMode extends LinearOpMode {
    public boolean manual = true;

    public static double move_XThrottle = 0.8;
    public static double move_YThrottle = 0.8;
    public static double move_RotateThrottle = 0.05;
    public static double move_decreaseRatio = 0.5;

    public static double intake1Flip_initDegree = 0;
    public static double intake1Flip_flatDegree = 355;

    public static long intake1Flip_m_cycleTime = 20;
    public static double intake1Flip_m_rotateInSec = 180;

    public static double intake1Spin_power = 0.8;

    public static double claw1_openDegree = 30;
    public static double claw1_closeDegree = 0;

    public static float vertSlide_maxPower_up = 1f;
    public static float vertSlide_maxPower_down = 1f;
    public static double vertSlide_PowerFactor_up = 1.2;
    public static double vertSlide_PowerFactor_down = 1.6;

    //Vertical Slider's Position Controller
    public static double vertSlide_kP = 0.002;
    public static double vertSlide_positionTolerance = 20;   // allowed maximum error
    public static double vertSlide_minPower = 0.3;

//    public static int vertDown_max = 20000;
//    public static int vertUp_max = 20000;

//    public static int vertUp_targetUp = 5000;
//    public static int vertUp_targetDown = 500;
//
//    public static int vertDown_targetUp = -5000;
//    public static int vertDown_targetDown = -500;

    /**
     * factor of extender power / retracter power when retracting
     */
    public static double intakeSlide_retractFactor = 1.2;

    public static double intakeSlide_joystickMaxPower = 0.5;

    private List<Action> runningActions = new ArrayList<>();

    private MecanumDrive mecanumDrive;
    private ServoDevice intake1FlipServo;
    private ServoDevice clawServo;
    private CRServo intake1SpinServo;

    MotorEx vertSlideUp;
    MotorEx vertSlideDown;

    private SlideManualMotor slideManualMotor;

    private HorizontalExtender horizontalExtender;

    private GamepadEx gamepadEx1;
    private GamepadEx gamepadEx2;

    private boolean decreasePowerTrain = true;
    private boolean useFieldCentricMecanumDrive = true;

    private long nextReadPoseTime;
    private double powerDecreaseFactor = 1.0;

    private final List<String> unavailableHardwares = new ArrayList<>();


    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        // obtain a list of hubs
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        initHardware();

        addActionEx(intake1FlipServo.manualRotate(
                () -> {
                    if (gamepad1.dpad_up) return -1f;
                    if (gamepad1.dpad_down) return 1f;
                    if (gamepad2.dpad_up) return -1f;
                    if (gamepad2.dpad_down) return 1f;
                    return 0f;
                },
                intake1Flip_m_cycleTime,
                intake1Flip_m_rotateInSec));

        addActionEx(slideManualMotor.runWithJoystick(
                () -> -gamepad2.left_stick_y,
                () -> -gamepad2.right_stick_y,
                () -> gamepadEx2.wasJustPressed(GamepadKeys.Button.BACK)));

        addActionEx(horizontalExtender.runWithJoystick(
                () -> {
                    if (gamepad1.left_trigger > 0.1) return gamepad1.left_trigger;
                    if (gamepad1.right_trigger > 0.1) return -gamepad1.right_trigger;
                    if (gamepad2.left_trigger > 0.1) return gamepad2.left_trigger;
                    if (gamepad2.right_trigger > 0.1) return -gamepad2.right_trigger;
                    return 0f;
                },
                intakeSlide_joystickMaxPower));


        waitForStart();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            gamepadEx1.readButtons();
            gamepadEx2.readButtons();

            // updated based on gamepads
            if (mecanumDrive.available) {
                if (gamepadEx1.wasJustPressed(GamepadKeys.Button.A)) {
                    decreasePowerTrain = !decreasePowerTrain;
                }

                if (gamepadEx1.wasJustPressed(GamepadKeys.Button.BACK)) {
                    useFieldCentricMecanumDrive = !useFieldCentricMecanumDrive;
                }

                double y = gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x;

                if (useFieldCentricMecanumDrive) {
                    mecanumDrive.pinpoint.update();
                    double botHeading = mecanumDrive.pinpoint.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                    double origX = x;
                    double origY = y;
                    // Rotate the movement direction counter to the bot's rotation
                    y = origX * Math.sin(botHeading) + origY * Math.cos(botHeading);
                    x = origX * Math.cos(botHeading) - origY * Math.sin(botHeading);
                }
                powerDecreaseFactor = decreasePowerTrain ? move_decreaseRatio : 1;
                mecanumDrive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                y * move_XThrottle * powerDecreaseFactor,
                                x * move_YThrottle * powerDecreaseFactor),
                        -gamepad1.right_stick_x * move_RotateThrottle * powerDecreaseFactor));

            }

            if (intake1SpinServo != null) {
                if (gamepadEx1.isDown(GamepadKeys.Button.LEFT_BUMPER) || gamepadEx2.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                    intake1SpinServo.set(-intake1Spin_power);
                } else if (gamepadEx1.isDown(GamepadKeys.Button.RIGHT_BUMPER) || gamepadEx2.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                    intake1SpinServo.set(intake1Spin_power);
                } else {
                    intake1SpinServo.set(0);
                }
            }

            if (gamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                vertSlideDown.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            } else if (gamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                vertSlideDown.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
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

            report(mecanumDrive, packet);
        }
    }

    private void initHardware() {
        List<ServoDevice> servoList = new ArrayList<>();

        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        intake1FlipServo = new ServoDevice(intake1FlipName, hardwareMap, telemetry, Math.max(intake1Flip_initDegree, intake1Flip_flatDegree));
        //intake1FlipServo.servo.setInverted(true);
        intake1FlipServo.setPracticalAngles(intake1Flip_initDegree, intake1Flip_flatDegree);
        servoList.add(intake1FlipServo);

        clawServo = new ServoDevice(clawName, hardwareMap, telemetry, 355);
        servoList.add(clawServo);

        try {
            intake1SpinServo = new CRServo(hardwareMap, intake1spinName);
            intake1SpinServo.setInverted(true);
        } catch (Exception e) {
            RobotLog.e("intake1SpinServo is not available");
        }

        try {
            vertSlideUp = new MotorEx(hardwareMap, vertSlideUpName, Motor.GoBILDA.RPM_435);
        } catch (Exception e) {
            RobotLog.e("VertSlideUp are not available");
        }

        try {
            vertSlideDown = new MotorEx(hardwareMap, vertSlideDownName, Motor.GoBILDA.RPM_312);
        } catch (Exception e) {
            RobotLog.e("VertSlideDown are not available");
        }

        slideManualMotor = new SlideManualMotor(vertSlideUp, vertSlideDown, telemetry);

        if (manual) {
            try {
                MotorEx horizontalExtenderMotor = new MotorEx(hardwareMap, horizontalSlide, Motor.GoBILDA.RPM_312);
                horizontalExtenderMotor.stopAndResetEncoder();

                //DON'T INVERT MOTORS AFTER HERE
                MotorGroupEx horizontalExtenderGroup = new MotorGroupEx(horizontalExtenderMotor);

                horizontalExtender = new HorizontalExtender(horizontalExtenderGroup, telemetry, "horizontalExtender");
            } catch (Exception e) {
                RobotLog.e("horizontalExtender are not available");
            }
        }


        //GAMEPADS
        if (manual) {
            gamepadEx1 = new GamepadEx(gamepad1);
            gamepadEx2 = new GamepadEx(gamepad2);
        }

        if (!mecanumDrive.available) {
            unavailableHardwares.add("MecanumDrive");
        }
        if (this.vertSlideUp == null) {
            unavailableHardwares.add("vertSlideUp");
        }
        if (this.vertSlideDown == null) {
            unavailableHardwares.add("vertSlideDown");
        }
        if (intake1SpinServo == null) {
            unavailableHardwares.add("intake1SpinServo");
        }

        if (!horizontalExtender.available)
            unavailableHardwares.add("horizontalExtender");

        unavailableHardwares.addAll(servoList.stream().filter(servo -> !servo.available).map(ServoDevice::getDeviceName).collect(Collectors.toList()));
    }

    private void report(MecanumDrive drive, TelemetryPacket packet) {
        long currentTime = System.currentTimeMillis();
        if (!unavailableHardwares.isEmpty()) {
            telemetry.addData("Unavailable devices", String.join(", ", unavailableHardwares));
        }

        if (drive.available) {
            if (currentTime > nextReadPoseTime) {
                drive.updatePoseEstimate();
                nextReadPoseTime = currentTime + 200;
            }
            telemetry.addData("drive mode", useFieldCentricMecanumDrive ? "Field Centric" : "Robot Centric");
            telemetry.addData("drive max power", move_XThrottle * powerDecreaseFactor);
            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.round(Math.toDegrees(drive.pose.heading.toDouble())));
        }

        if (intake1SpinServo != null) {
            telemetry.addData("intake1Spin power", intake1SpinServo.get());
        }

        if (clawServo.available) {
            telemetry.addData("clawServo Angle", clawServo.getAngle());
        }

        telemetry.update();

        packet.fieldOverlay().setStroke("#3F51B5");
        if (drive.available) {
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
        }
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
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

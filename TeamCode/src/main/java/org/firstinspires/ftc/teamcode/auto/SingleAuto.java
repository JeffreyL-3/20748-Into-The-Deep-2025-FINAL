package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.ANewActionTeleOpMode.intake1Flip_flatDegree;
import static org.firstinspires.ftc.teamcode.ANewActionTeleOpMode.intake1Flip_initDegree;
import static edu.nobles.robotics.parameters.DeviceNameList.clawName;
import static edu.nobles.robotics.parameters.DeviceNameList.intake1FlipName;
import static edu.nobles.robotics.parameters.DeviceNameList.intake1spinName;
import static edu.nobles.robotics.parameters.DeviceNameList.vertSlideDownName;
import static edu.nobles.robotics.parameters.DeviceNameList.vertSlideUpName;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.ANewActionTeleOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import edu.nobles.robotics.ActionEx;
import edu.nobles.robotics.motor.SlideActionMotor;
import edu.nobles.robotics.parameters.ParameterManager;
import edu.nobles.robotics.servo.ServoDevice;


@Config
@Autonomous(name = "SingleAuto", group = "Autonomous")
public class SingleAuto extends LinearOpMode {
    public boolean manual = false; //CHANGE IN AUTO

    public static double startX = 36;
    public static double startY = -62.5;
    public static double startHeading = -90;

    public static double firstX = -90;
    public static double targetY = -90;

    //start pose
    private Pose2d beginPose = new Pose2d(14, -63, Math.toRadians(-90));

    private MecanumDrive mecanumDrive;
    private ServoDevice intake1FlipServo;
    private ServoDevice clawServo;
    private CRServo intake1SpinServo;
    private SlideActionMotor vertSlide;
    private boolean vertSlideAlreadyStopped;

    private SlideActionMotor horizontalExtender;

    private final List<String> unavailableHardwares = new ArrayList<>();

    @Override
    public void runOpMode() {

        initHardware();

        TrajectoryActionBuilder moveInitialTraj = mecanumDrive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(0, -43));
                //Check for vertical slide extension

        TrajectoryActionBuilder moveToSubTraj = moveInitialTraj.endTrajectory().fresh()
                .strafeTo(new Vector2d(0, -34));
                //Retract vertical slide, snapping piece onto chamber
                //Open claw

        TrajectoryActionBuilder movePickupTraj = moveToSubTraj.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(47, -60), Math.toRadians(-90));
                //Wait
                //Close claw
        TrajectoryActionBuilder moveInital2Traj = moveToSubTraj.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(0, -43), Math.toRadians(90));
                //Check for vertical slide extension
        TrajectoryActionBuilder pushTraj = moveToSubTraj.endTrajectory().fresh()
                .strafeTo(new Vector2d(36, -34))
                .strafeTo(new Vector2d(36, -9))
                .splineToConstantHeading(new Vector2d(46, -8), Math.toRadians(-90))
                .splineTo(new Vector2d(46, -60), Math.toRadians(-90))
                .strafeTo(new Vector2d(46, -9))
                .splineToConstantHeading(new Vector2d(58, -9), Math.toRadians(-90))
                .splineTo(new Vector2d(58, -60), Math.toRadians(-90));

                /*
                .strafeTo(new Vector2d(56, 0))
                .strafeTo(new Vector2d(65, 0))
                .strafeTo(new Vector2d(65, -60));
                 */

        Action moveInitial = moveInitialTraj.build();
        Action moveToSub = moveToSubTraj.build();
        Action movePickup = movePickupTraj.build();
        Action moveInital2 = moveInital2Traj.build();


        Action push = pushTraj.build();

        // actions that need to happen on init; for instance, a claw tightening.
        // INSERT HERE

        clawServo.rotateNormal(ANewActionTeleOpMode.claw1_closeDegree);

        waitForStart();

        if (isStopRequested()) return;


        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                new ParallelAction(
                                        moveInitial,
                                        vertSlide.moveSlide(ParameterManager.highChamberExtendUpPos)
                                ),
                                moveToSub,
                                new SleepAction(1.0),
                                new ParallelAction(
                                        vertSlide.moveSlide(ParameterManager.highChamberRetractUpPos)
                                ),

                                //TODO: May need race action (new, need to merge) with timer to open claw even if motors stall
                                clawServo.rotateNormal(ANewActionTeleOpMode.claw1_openDegree),
                                new SleepAction(1.0),
                                new ParallelAction(
                                        vertSlide.moveSlide(ParameterManager.wallUpPos)
                                )//,
                                //push
                        ),
                        createReportAction()
                )
        );
    }

    private void initHardware() {
        List<ServoDevice> servoList = new ArrayList<>();

        mecanumDrive = new MecanumDrive(hardwareMap, beginPose);

        intake1FlipServo = new ServoDevice(intake1FlipName, hardwareMap, telemetry);
        intake1FlipServo.setPracticalAngles(intake1Flip_initDegree, intake1Flip_flatDegree);
        intake1FlipServo.rotateNormal(intake1Flip_initDegree);
        servoList.add(intake1FlipServo);

        clawServo = new ServoDevice(clawName, hardwareMap, telemetry, 255);
        servoList.add(clawServo);

        try {
            intake1SpinServo = new CRServo(hardwareMap, intake1spinName);
            intake1SpinServo.setInverted(true);
        } catch (Exception e) {
            RobotLog.e("intake1SpinServo is not available");
        }

        try {
            MotorEx vertSlideUp = new MotorEx(hardwareMap, vertSlideUpName, Motor.GoBILDA.RPM_435);
            MotorEx vertSlideDown = new MotorEx(hardwareMap, vertSlideDownName, Motor.GoBILDA.RPM_312);
            //DON'T INVERT MOTORS AFTER HERE
            vertSlide = new SlideActionMotor(vertSlideUp, vertSlideDown, telemetry, "vertSlideUp", -1);

        } catch (Exception e) {
            RobotLog.e("VertSlideUp are not available");
        }

        if (!mecanumDrive.available) {
            unavailableHardwares.add("MecanumDrive");
        }
        if (vertSlide == null) {
            unavailableHardwares.add("vertSlide");
        }
        if (intake1SpinServo == null) {
            unavailableHardwares.add("intake1SpinServo");
        }

        unavailableHardwares.addAll(servoList.stream().filter(servo -> !servo.available).map(ServoDevice::getDeviceName).collect(Collectors.toList()));
    }

    ActionEx createReportAction() {
        return new ActionEx() {
            @Override
            public String getDeviceName() {
                return "ReportAction";
            }

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                telemetry.addData("VerSlide Position", vertSlide.slideUpMotor.getCurrentPosition());
                telemetry.addData("Claw Angle", clawServo.getAngle());

                telemetry.update();
                return true;
            }
        };
    }

}

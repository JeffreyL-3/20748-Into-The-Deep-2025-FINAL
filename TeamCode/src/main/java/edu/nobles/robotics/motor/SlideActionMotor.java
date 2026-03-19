package edu.nobles.robotics.motor;

import static org.firstinspires.ftc.teamcode.ANewActionTeleOpMode.*;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ANewActionTeleOpMode;

import edu.nobles.robotics.ActionEx;


@Config
public class SlideActionMotor {
    private final String deviceName;
    private final Telemetry telemetry;
    public final MotorEx slideUpMotor;
    private final MotorEx slideDownMotor;

    public static long stopTime = 4000;

    public SlideActionMotor(MotorEx upSlideMotor, MotorEx slideDownMotor, Telemetry telemetry, String deviceName, double stallCurrent) {
        this.slideUpMotor = upSlideMotor;
        this.slideDownMotor = slideDownMotor;
        this.deviceName = deviceName;
        this.telemetry = telemetry;
        slideUpMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        slideUpMotor.stopAndResetEncoder();
        slideUpMotor.set(0);
    }

    public void setManualMode() {
        RobotLog.i(deviceName + " setManualMode ");
        slideUpMotor.setRunMode(Motor.RunMode.RawPower);
    }

    public void setActionMode() {
        RobotLog.i(deviceName + " setActionMode ");
        slideUpMotor.setRunMode(Motor.RunMode.PositionControl);
        slideUpMotor.setPositionCoefficient(vertSlide_kP);
        slideUpMotor.setPositionTolerance(vertSlide_positionTolerance); // allowed maximum error
    }

    public void zeroPowerWithFloat() {
        slideUpMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        slideUpMotor.set(0);
    }

    public void zeroPowerWithBrake() {
        slideUpMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        slideUpMotor.set(0);
    }

    public String getDeviceName() {
        return deviceName;
    }

    //Universal MoveSlide Action (doesn't care about direction)

    public class PosMoveSlideAction implements ActionEx {
        int targetPosition;    //int for desired tick count
        double lastPowerSet = 0;
        boolean inited = false;
        long stuckStartTime;
        int prePosition;
        long preTime;

        public PosMoveSlideAction(int targetPosition) {
            this.targetPosition = targetPosition;
        }

        public String getDeviceName() {
            return deviceName;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            int current = slideUpMotor.getCurrentPosition();
            long currentTime = System.currentTimeMillis();
            if (!inited) {
                prePosition = current;
                preTime = currentTime;

                slideUpMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
                setActionMode();
                slideUpMotor.setTargetPosition(targetPosition);

                RobotLog.i(deviceName + " Initial Position: " + current);
                RobotLog.i(deviceName + " To Position: " + targetPosition);
                telemetry.addData(deviceName + " To Position", targetPosition);

                inited = true;
            }

            telemetry.addData(deviceName + " Current Position", current);
            RobotLog.i(deviceName + " current position:" + current);

            double velocity = currentTime == preTime ? 1000 : ((double) (current - prePosition)) / (currentTime - preTime);
            RobotLog.i(String.format("%s velocity:%.3f", deviceName, velocity));

            if (Math.abs(velocity) < 0.03) {
                stuckStartTime = currentTime;
                RobotLog.i("stuck start");
            }

            boolean stopBecauseStuck = (stuckStartTime >= 1) && (currentTime - stuckStartTime > stopTime);
            boolean inEndRange = Math.abs(current - targetPosition) <= vertSlide_positionTolerance;

            if ((slideUpMotor.atTargetPosition() && inEndRange) || stopBecauseStuck) {
                slideUpMotor.stopMotor();
                slideDownMotor.stopMotor();

                RobotLog.i(deviceName + " Stop");
                telemetry.addLine(deviceName + " Stop");
                lastPowerSet = 0;
                return false;
            }

            if (targetPosition >= current)
                slideUpMotor.set(vertSlide_maxPower_up);
            else
                slideUpMotor.set(vertSlide_maxPower_down);

            double actualPower = slideUpMotor.get();
            if (Math.abs(actualPower) < vertSlide_minPower) {
                actualPower = Math.signum(actualPower) * vertSlide_minPower;
                slideUpMotor.motor.setPower(actualPower);
            }

            double downPower = actualPower;
            downPower *= actualPower > 0 ? ANewActionTeleOpMode.vertSlide_PowerFactor_up : ANewActionTeleOpMode.vertSlide_PowerFactor_down;
            downPower *= ANewActionTeleOpMode.vertSlide_maxPower_down;
            slideDownMotor.set(downPower);

            RobotLog.i(deviceName + " actual Power:" + actualPower);
            telemetry.addData(deviceName + " Power", actualPower);

            if (currentTime - preTime >= 1000) {
                prePosition = current;
                preTime = currentTime;
            }
            return true;
        }
    }

    public ActionEx moveSlide(int targetPosition) {
        return new PosMoveSlideAction(targetPosition);
    }
}

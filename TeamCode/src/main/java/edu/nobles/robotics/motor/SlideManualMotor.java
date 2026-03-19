package edu.nobles.robotics.motor;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ANewActionTeleOpMode;

import java.util.function.Supplier;

import edu.nobles.robotics.ActionEx;

public class SlideManualMotor {
    public static double ignoreCutoff = 0.05;

    enum Mode {UpOnly, UpDown}

    private final MotorEx slideUpMotor;
    private final MotorEx slideDownMotor;
    private final Telemetry telemetry;

    Mode mode = Mode.UpDown;
    ZeroPowerBehavior zeroPowerBehavior;


    public SlideManualMotor(MotorEx slideUpMotor, MotorEx slideDownMotor, Telemetry telemetry) {
        this.slideUpMotor = slideUpMotor;
        this.slideDownMotor = slideDownMotor;
        this.telemetry = telemetry;
    }

    void setUpPower(double power) {
        if (slideUpMotor != null) {
            power *= ANewActionTeleOpMode.vertSlide_maxPower_up;
            slideUpMotor.set(power);
        }
    }

    void setDownPower(double power) {
        if (slideDownMotor != null && (mode == Mode.UpDown)) {
            power *= ANewActionTeleOpMode.vertSlide_maxPower_down;
            slideDownMotor.set(power);
        }
    }

    public void setPower(double power) {
        if (Math.abs(power) < ignoreCutoff) {
            setUpPower(0);
            setDownPower(0);
            setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        } else if (power > 0) {
            setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
            setUpPower(power);
            setDownPower(power * ANewActionTeleOpMode.vertSlide_PowerFactor_up);
        } else {
            setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
            setUpPower(power);
            setDownPower(power * ANewActionTeleOpMode.vertSlide_PowerFactor_down);
        }
    }

    public void setDownSliderPower(double power) {
        if (slideDownMotor == null)
            return;

        if (Math.abs(power) < ignoreCutoff) {
            slideDownMotor.set(0);
        } else {
            if (slideUpMotor != null)
                slideUpMotor.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
            slideDownMotor.set(power * ANewActionTeleOpMode.vertSlide_maxPower_down);
        }
    }

    private void setZeroPowerBehavior(ZeroPowerBehavior target) {
        if (zeroPowerBehavior != target) {
            zeroPowerBehavior = target;
            if (slideUpMotor != null)
                slideUpMotor.setZeroPowerBehavior(target);
            if (slideDownMotor != null)
                slideDownMotor.setZeroPowerBehavior(target);
        }
    }

    void changeMode() {
        switch (mode) {
            case UpOnly:
                mode = Mode.UpDown;
                break;
            case UpDown:
                mode = Mode.UpOnly;
                break;
        }
    }

    public class SlideManualMotorWithJoystickAction implements ActionEx {
        Supplier<Float> upSliderControl;
        Supplier<Float> downSliderControl;
        Supplier<Boolean> modeChange;
        boolean inited;

        public SlideManualMotorWithJoystickAction(Supplier<Float> upSliderControl, Supplier<Float> downSliderControl, Supplier<Boolean> modeChange) {
            this.upSliderControl = upSliderControl;
            this.downSliderControl = downSliderControl;
            this.modeChange = modeChange;
        }

        public String getDeviceName() {
            return "SlideManualMotor";
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!inited) {
                if (slideUpMotor != null) {
                    slideUpMotor.setRunMode(Motor.RunMode.RawPower);
                    slideUpMotor.resetEncoder();
                }
                if (slideDownMotor != null) {
                    slideDownMotor.setRunMode(Motor.RunMode.RawPower);
                    slideDownMotor.resetEncoder();
                }
                inited = true;
            }

            if (modeChange.get()) {
                changeMode();
            }

            setPower(upSliderControl.get());

            if (Math.abs(upSliderControl.get()) < ignoreCutoff) {
                setDownSliderPower(downSliderControl.get());
            }

            telemetry.addData("VertSlide mode", mode.name());
            if (slideUpMotor != null)
                telemetry.addData("VertSlide Up position", slideUpMotor.getCurrentPosition());
            if (slideDownMotor != null)
                telemetry.addData("VertSlide Down position", slideDownMotor.getCurrentPosition());

            return true;
        }
    }

    public ActionEx runWithJoystick(Supplier<Float> upSliderControl, Supplier<Float> downSliderControl, Supplier<Boolean> modeChange) {
        return new SlideManualMotorWithJoystickAction(upSliderControl, downSliderControl, modeChange);
    }

}

package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.auto.DoubleAuto.startAngle;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Resets {
    private Pose2d newPose = new Pose2d(0, -63, Math.toRadians(startAngle));
    private MecanumDrive mecanumDrive;


    public Resets(MecanumDrive mecanumDrive, Pose2d newPose) {
        this.mecanumDrive=mecanumDrive;
        this.newPose=newPose;
    }

    public class resetPinpointAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            mecanumDrive.pinpoint.setPosition(newPose);
            return false;
        }
    }

    public Action resetPinpoint() {
        return new resetPinpointAction();
    }
}

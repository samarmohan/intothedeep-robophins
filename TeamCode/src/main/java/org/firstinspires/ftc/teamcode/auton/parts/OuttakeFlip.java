package org.firstinspires.ftc.teamcode.auton.parts;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeFlip {
    private final Servo outtakeFlip;

    public OuttakeFlip(HardwareMap hardwareMap) {
        outtakeFlip = hardwareMap.get(Servo.class, "Outtake Flip");
    }

    public Action outtakeFlipOut() {
        return new OuttakeFlipOut();
    }

    public Action outtakeFlipIn() {
        return new OuttakeFlipIn();
    }

    public class OuttakeFlipOut implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            outtakeFlip.setPosition(0.4);
            return false;
        }
    }

    public class OuttakeFlipIn implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            outtakeFlip.setPosition(1);
            return false;
        }
    }

}

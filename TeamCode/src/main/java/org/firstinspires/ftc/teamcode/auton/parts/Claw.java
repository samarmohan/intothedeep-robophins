package org.firstinspires.ftc.teamcode.auton.parts;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private final Servo outtakeClaw;

    public Claw(HardwareMap hardwareMap) {
        outtakeClaw = hardwareMap.get(Servo.class, "Outtake Grasp");
    }

    public Action clawClose() {
        return new ClawClose();
    }

    public Action clawOpen() {
        return new ClawOpen();
    }

    public class ClawClose implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            outtakeClaw.setPosition(0.4);
            return false;
        }
    }

    public class ClawOpen implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            outtakeClaw.setPosition(0.3);
            return false;
        }
    }

}

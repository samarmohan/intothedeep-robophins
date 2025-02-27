package org.firstinspires.ftc.teamcode.auton.parts;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSlides {
    DcMotor intakeSlide;

    public IntakeSlides(HardwareMap hardwareMap) {
        intakeSlide = hardwareMap.get(DcMotor.class, "Intake Slide");
    }

    public Action keepIntakeIn() {
        return new KeepIntakeIn();
    }

    public class KeepIntakeIn implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeSlide.setPower(0.1);

            int ispos = intakeSlide.getCurrentPosition();

            packet.put("intake slides position", ispos);

            return (ispos < 0);
        }
    }
}

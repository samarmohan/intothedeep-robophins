package org.firstinspires.ftc.teamcode.auton.parts;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class VerticalSlides {

    private final DcMotor leftSlide;
    private final DcMotor rightSlide;

    public VerticalSlides(HardwareMap hardwareMap) {
        leftSlide = hardwareMap.get(DcMotor.class, "Left Slide");
        rightSlide = hardwareMap.get(DcMotor.class, "Right Slide");

        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public Action slidesUp(int x) {
        return new SlidesUp(x);
    }

    public Action slidesDown(int x) {
        return new SlidesDown(x);
    }

    public class SlidesUp implements Action {
        private final int toPos;

        public SlidesUp(int x) {
            toPos = x;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            leftSlide.setPower(1);
            rightSlide.setPower(1);

            int lsPosition = leftSlide.getCurrentPosition();
            int rsPosition = rightSlide.getCurrentPosition();

            packet.put("left slide position", lsPosition);
            packet.put("right slide position", rsPosition);

            if (lsPosition < toPos || rsPosition < toPos) {
                return true;
            } else {
                leftSlide.setPower(0.1);
                rightSlide.setPower(0.1);
                return false;
            }
        }
    }

    public class SlidesDown implements Action {

        private final int toPos;
        public SlidesDown(int x) {
            toPos = x;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            leftSlide.setPower(-1);
            rightSlide.setPower(-1);

            int lsPosition = leftSlide.getCurrentPosition();
            int rsPosition = rightSlide.getCurrentPosition();

            packet.put("left slide position", lsPosition);
            packet.put("right slide position", rsPosition);

            if (lsPosition > toPos || rsPosition > toPos) {
                return true;
            } else {
                return false;
            }
        }
    }
}

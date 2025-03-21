package org.firstinspires.ftc.teamcode.auton;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;

@Autonomous(name = "AutonTest", group = "Specimen")
public class AutonTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(-90));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, beginPose);
        Slides slides = new Slides(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        OuttakeFlip outtakeFlip = new OuttakeFlip(hardwareMap);
        IntakeSlide intakeSlide = new IntakeSlide(hardwareMap);

        int SLIDES_UP = 1725;
        int SLIDES_GRAB = 80;

        // close claw on init
        Actions.runBlocking(new SequentialAction(claw.clawClose()));

        Action action = new ParallelAction(
                slides.slidesUp(100),
                intakeSlide.keepIntakeIn(),
                drive.actionBuilder(beginPose)
                        .strafeTo(new Vector2d(0, 20))
                        .afterTime(0, slides.slidesUp(1700))
                        .strafeTo(new Vector2d(70, 0))
                        .build()
        );

        waitForStart();
        telemetry.update();
        if (isStopRequested()) return;

        Actions.runBlocking(
                action
        );

        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    public static class OuttakeFlip {
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
                outtakeFlip.setPosition(0);
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

    public static class Claw {
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

    public static class Slides {

        private final DcMotor leftSlide;
        private final DcMotor rightSlide;

        public Slides(HardwareMap hardwareMap) {
            leftSlide = hardwareMap.get(DcMotor.class, "Left Slide");
            rightSlide = hardwareMap.get(DcMotor.class, "Right Slide");

            leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);

            leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

                packet.put("leftS", lsPosition);
                packet.put("rspos", rsPosition);

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

                packet.put("lspos", lsPosition);
                packet.put("rspos", rsPosition);

                if (lsPosition > toPos || rsPosition > toPos) {
                    return true;
                } else {
                    return false;
                }
            }
        }
    }

    public static class IntakeSlide {
        DcMotor intakeSlide;

        public IntakeSlide(HardwareMap hardwareMap) {
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

                packet.put("isPos", ispos);

                return (ispos < 0);
            }
        }
    }
}
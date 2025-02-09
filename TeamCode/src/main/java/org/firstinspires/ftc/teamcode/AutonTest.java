package org.firstinspires.ftc.teamcode;

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

@Autonomous(name = "AutonTest", group = "Test")
public class AutonTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(-90));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, beginPose);
        Slides slides = new Slides(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        OuttakeFlip outtakeFlip = new OuttakeFlip(hardwareMap);
        IntakeFlip intakeFlip = new IntakeFlip(hardwareMap);
        IntakeSlide intakeSlide = new IntakeSlide(hardwareMap);


        Action test = new ParallelAction(
                slides.slidesUp(-1700),
                drive.actionBuilder(beginPose)
                        .afterTime(1, slides.slidesUp(-1700))
                        .afterTime(0.5, outtakeFlip.outtakeFlipIn())
                        .strafeTo(new Vector2d(0, 40))
                        .strafeTo(new Vector2d(0, 10))
                        .afterTime(0.5, slides.slidesDown(0))
                        .strafeTo(new Vector2d(20, 10))
                        .build()
        );

        waitForStart();
        telemetry.update();
        if (isStopRequested()) return;

        Actions.runBlocking(
                test
        );

        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    public static class IntakeFlip {
        Servo intakeFlipRight;
        Servo intakeFlipLeft;

        public IntakeFlip(HardwareMap hardwareMap) {
            intakeFlipRight = hardwareMap.get(Servo.class, "Intake Flip Right");
            intakeFlipLeft = hardwareMap.get(Servo.class, "Intake Flip Left");
        }

        public Action intakeFlipUp() {
            return new IntakeFlipUp();
        }

        public class IntakeFlipUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeFlipRight.setPosition(0.5);
                intakeFlipLeft.setPosition(0.5);
                return false;
            }
        }
    }

    public static class OuttakeFlip {
        private final Servo outtakeFlip;

        public OuttakeFlip(HardwareMap hardwareMap) {
            outtakeFlip = hardwareMap.get(Servo.class, "Outtake Flip");
        }

        public Action outtakeFlipOpen() {
            return new OuttakeFlipOpen();
        }

        public Action outtakeFlipIn() {
            return new OuttakeFlipIn();
        }

        public class OuttakeFlipOpen implements Action {
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

                // TODO tune for proper position
                if (lsPosition > toPos || rsPosition > toPos) {
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

                return lsPosition < toPos || rsPosition < toPos;
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
                intakeSlide.setPower(0.2);

                int ispos = intakeSlide.getCurrentPosition();

                packet.put("isPos", ispos);

                return (ispos < 0);
            }
        }
    }
}
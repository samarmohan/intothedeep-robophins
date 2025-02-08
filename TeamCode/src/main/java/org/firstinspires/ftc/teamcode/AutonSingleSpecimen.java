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

@Autonomous(name="AutonSingleSpecimen", group="Specimen")
public class AutonSingleSpecimen extends LinearOpMode {
    public static class IntakeFlip {
        Servo intakeFlipRight;
        Servo intakeFlipLeft;

        public IntakeFlip(HardwareMap hardwareMap) {
            intakeFlipRight = hardwareMap.get(Servo.class, "Intake Flip Right");
            intakeFlipLeft = hardwareMap.get(Servo.class, "Intake Flip Left");
        }

        public class IntakeFlipUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeFlipRight.setPosition(0.5);
                intakeFlipLeft.setPosition(0.5);
                return false;
            }
        }

        public Action intakeFlipUp() {
            return new IntakeFlipUp();
        }
    }

    public static class OuttakeFlip {
        private final Servo outtakeFlip;
        public OuttakeFlip(HardwareMap hardwareMap) {
            outtakeFlip = hardwareMap.get(Servo.class, "Outtake Flip");
        }

        public class OuttakeFlipOpen implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtakeFlip.setPosition(0);
                return false;
            }
        }

        public Action outtakeFlipOpen() {
            return new OuttakeFlipOpen();
        }

        public class OuttakeFlipIn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtakeFlip.setPosition(1);
                return false;
            }
        }

        public Action outtakeFlipIn() {
            return new OuttakeFlipIn();
        }

    }

    public static class Claw {
        private final Servo outtakeClaw;
        public Claw(HardwareMap hardwareMap) {
            outtakeClaw = hardwareMap.get(Servo.class, "Outtake Grasp");
        }

        public class ClawClose implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtakeClaw.setPosition(0.4);
                return false;
            }
        }

        public Action clawClose() {
            return new ClawClose();
        }

        public class ClawOpen implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtakeClaw.setPosition(0.3);
                return false;
            }
        }

        public Action clawOpen() {
            return new ClawOpen();
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
                }
                else {
                    leftSlide.setPower(0.1);
                    rightSlide.setPower(0.1);
                    return false;
                }
            }
        }

        public Action slidesUp(int x) {
            return new SlidesUp(x);
        }

        public class SlidesDown implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftSlide.setPower(-1);
                rightSlide.setPower(-1);

                int lsPosition = leftSlide.getCurrentPosition();
                int rsPosition = rightSlide.getCurrentPosition();

                packet.put("lspos", lsPosition);
                packet.put("rspos", rsPosition);

                return lsPosition < 0 || rsPosition < 0;
            }
        }

        public Action slidesDown() {
            return new SlidesDown();
        }
    }

    public static class IntakeSlide {
        DcMotor intakeSlide;

        public IntakeSlide(HardwareMap hardwareMap) {
            intakeSlide = hardwareMap.get(DcMotor.class, "Intake Slide");
        }

        public class KeepIntakeIn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeSlide.setPower(0.2);

                int ispos = intakeSlide.getCurrentPosition();

                packet.put("isPos", ispos);

                return (ispos <0);
            }
        }

        public Action keepIntakeIn() {
            return new KeepIntakeIn();
        }
    }

    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(-90));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, beginPose);
        Slides slides = new Slides(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        OuttakeFlip outtakeFlip = new OuttakeFlip(hardwareMap);
        IntakeFlip intakeFlip = new IntakeFlip(hardwareMap);
        IntakeSlide intakeSlide = new IntakeSlide(hardwareMap);

        int SLIDES_UP = -1750;

        // close claw on init
        Actions.runBlocking(new SequentialAction(claw.clawClose(), intakeFlip.intakeFlipUp()));

        Action oneSpecimen = new ParallelAction(
                slides.slidesUp(SLIDES_UP),
                intakeSlide.keepIntakeIn(),
                drive.actionBuilder(beginPose)
                        // first specimen
                        // wait for slides up
                        .waitSeconds(0.5)
                        .stopAndAdd(outtakeFlip.outtakeFlipOpen())
                        // place preloaded specimen on bar
                        .strafeTo(new Vector2d(0, 33))
                        .stopAndAdd(claw.clawOpen())
                        // move back and away from submersible
                        .strafeTo(new Vector2d(0, 20))
                        .stopAndAdd(slides.slidesDown())
                        // park
                        .strafeTo(new Vector2d(40, 10))
                        // cleanup
                        .stopAndAdd(outtakeFlip.outtakeFlipIn())
                        .waitSeconds(1)
                        .stopAndAdd(slides.slidesDown())
                        .build()
        );

        waitForStart();
        telemetry.update();
        if (isStopRequested()) return;

        Actions.runBlocking(
                oneSpecimen
        );

        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="AutonSingleSpecimen", group="Specimen")
public class AutonSingleSpecimen extends LinearOpMode {
    public class Flip {
        private Servo outtakeFlip;
        public Flip(HardwareMap hardwareMap) {
            outtakeFlip = hardwareMap.get(Servo.class, "Outtake Flip");
        }

        public class FlipClose implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtakeFlip.setPosition(0);
                return false;
            }
        }

        public Action flipClose() {
            return new FlipClose();
        }

        public class FlipIn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtakeFlip.setPosition(1);
                return false;
            }
        }

        public Action flipIn() {
            return new FlipIn();
        }

    }

    public class Claw {
        private Servo outtakeClaw;
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

    public class Slides {

        private DcMotor LS;
        private DcMotor RS;

        public Slides(HardwareMap hardwareMap) {
            LS = hardwareMap.get(DcMotor.class, "Left Slide");
            RS = hardwareMap.get(DcMotor.class, "Right Slide");

            LS.setDirection(DcMotorSimple.Direction.REVERSE);

//            LS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            RS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//            LS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            RS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }

        public class SlidesUp implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                LS.setPower(1);
                RS.setPower(1);

                int lsPosition = LS.getCurrentPosition();
                int rsPosition = RS.getCurrentPosition();

                packet.put("lspos", lsPosition);
                packet.put("rspos", rsPosition);

                if (lsPosition > -1750 || rsPosition > -1750) {
                    return true;
                }
                else {
                    LS.setPower(0.1);
                    RS.setPower(0.1);
                    return false;
                }
            }
        }

        public Action slidesUp() {
            return new SlidesUp();
        }
    }


    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, beginPose);
        Slides slides = new Slides(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Flip flip = new Flip(hardwareMap);

        Actions.runBlocking(claw.clawClose());

        Action forward = drive.actionBuilder(beginPose)
                .lineToX(-30)
                .build();

        Action back = drive.actionBuilder(beginPose)
                .lineToX(10)
                .build();

        Action wait = drive.actionBuilder(beginPose).waitSeconds(2).build();
        Action wait15 = drive.actionBuilder(beginPose).waitSeconds(15).build();


        waitForStart();
        telemetry.update();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(slides.slidesUp(),
                    new SequentialAction(
                            flip.flipClose(),
                            forward,
                            wait,
                            claw.clawOpen(),
                            back,
                            claw.clawClose(),
                            flip.flipIn(),
                            wait
                    )
                )
        );
    }
}
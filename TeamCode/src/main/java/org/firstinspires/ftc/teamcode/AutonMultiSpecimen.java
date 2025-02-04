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

@Autonomous(name="AutonMultiSpecimen", group="Specimen")
public class AutonMultiSpecimen extends LinearOpMode {
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
        Pose2d beginPose = new Pose2d(10, -62, Math.PI / 2 );
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, beginPose);
        Slides slides = new Slides(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Flip flip = new Flip(hardwareMap);

        // close claw on init
        Actions.runBlocking(claw.clawClose());

        Action toSubmersible = drive.actionBuilder(beginPose)
                .lineToY(-30)
                .build();

        Action backUp = drive.actionBuilder(beginPose)
                .lineToY(10)
                .build();

        Action wait2 = drive.actionBuilder(beginPose).waitSeconds(2).build();

        Action placeSpecimen = new ParallelAction(
                slides.slidesUp(),
                new SequentialAction(
                        flip.flipClose(),
                        toSubmersible,
                        wait2,
                        claw.clawOpen(),
                        backUp,
                        claw.clawClose(),
                        wait2,
                        backUp
                )
        );

        Action pusher = drive.actionBuilder(beginPose)
                //.splineTo(new Vector2d(38, -13), Math.PI /2 )
                .strafeTo(new Vector2d(40, -62))
//                .strafeTo(new Vector2d(47, -53))
//                .strafeTo(new Vector2d(47, -13))
//                .strafeTo(new Vector2d(56, -13))
//                .strafeTo(new Vector2d(56, -53))
//                .strafeTo(new Vector2d(56, -13))
//                .strafeTo(new Vector2d(65, -13))
//                .strafeTo(new Vector2d(65, -53))
                .build();

        waitForStart();
        telemetry.update();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        //placeSpecimen,
                        pusher
                )
        );

        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
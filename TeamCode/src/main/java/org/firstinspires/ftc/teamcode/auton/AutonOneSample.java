package org.firstinspires.ftc.teamcode.auton;

import static org.firstinspires.ftc.teamcode.auton.Constants.SLIDES_GRAB;
import static org.firstinspires.ftc.teamcode.auton.Constants.SLIDES_UP;
import static org.firstinspires.ftc.teamcode.auton.Constants.WAIT_TO_ALIGN;
import static org.firstinspires.ftc.teamcode.auton.Constants.WAIT_TO_GRAB;

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

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.auton.parts.Claw;
import org.firstinspires.ftc.teamcode.auton.parts.IntakeSlides;
import org.firstinspires.ftc.teamcode.auton.parts.OuttakeFlip;
import org.firstinspires.ftc.teamcode.auton.parts.VerticalSlides;

@Autonomous(name = "AutonOneSample", group = "Sample")
public class AutonOneSample extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(90));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, beginPose);
        VerticalSlides verticalSlides = new VerticalSlides(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        OuttakeFlip outtakeFlip = new OuttakeFlip(hardwareMap);
        IntakeSlides intakeSlides = new IntakeSlides(hardwareMap);

        // close claw on init
        Actions.runBlocking(new SequentialAction(claw.clawClose()));

        Action action = new ParallelAction(
                verticalSlides.slidesUp(4350),
                intakeSlides.keepIntakeIn(),
                drive.actionBuilder(beginPose)
                        // REMEMBER COORDINATES IN DIFFERENT FRAME OF REFERENCE
                        .waitSeconds(0.5)
                        .strafeTo(new Vector2d(10, -5))
                        .turnTo(Math.toRadians(225))
                        .stopAndAdd(outtakeFlip.outtakeFlipOut())
                        .strafeTo(new Vector2d(25, -7))
                        .waitSeconds(1)
                        .stopAndAdd(claw.clawOpen())
                        .waitSeconds(1)
                        .strafeTo(new Vector2d(13, -20))
                        // cleanup
                        .stopAndAdd(claw.clawClose())
                        .stopAndAdd(outtakeFlip.outtakeFlipIn())
                        .waitSeconds(1)
                        .stopAndAdd(verticalSlides.slidesDown(0))
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
}
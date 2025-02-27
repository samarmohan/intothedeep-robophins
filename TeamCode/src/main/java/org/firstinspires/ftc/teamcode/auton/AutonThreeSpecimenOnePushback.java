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

@Autonomous(name = "AutonThreeSpecimenOnePushback", group = "Specimen")
public class AutonThreeSpecimenOnePushback extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(-90));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, beginPose);
        VerticalSlides verticalSlides = new VerticalSlides(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        OuttakeFlip outtakeFlip = new OuttakeFlip(hardwareMap);
        IntakeSlides intakeSlides = new IntakeSlides(hardwareMap);

        // close claw on init
        Actions.runBlocking(new SequentialAction(claw.clawClose()));

        Action action = new ParallelAction(
                verticalSlides.slidesUp(SLIDES_UP),
                intakeSlides.keepIntakeIn(),
                drive.actionBuilder(beginPose)
                        // first specimen
                        // wait for verticalSlides up
                        .waitSeconds(0.5)
                        .afterTime(0, outtakeFlip.outtakeFlipOut())
                        // place preloaded specimen on bar
                        .strafeTo(new Vector2d(0, 30))
                        .afterTime(0, claw.clawOpen())
                        // move back and away from submersible
                        .strafeTo(new Vector2d(0, 20))
                        .afterTime(0, verticalSlides.slidesDown(0))
                        .strafeToSplineHeading(new Vector2d(35, 20), Math.toRadians(0))
                        // go past samples
                        .strafeTo(new Vector2d(35, 52))
                        // move forward in front of sample
                        .strafeTo(new Vector2d(43, 52))
                        // push sample back to human player and turn
                        .strafeTo(new Vector2d(55, 13))
                        .turnTo(Math.toRadians(90))
                        // go back out of human player zone
                        .strafeTo(new Vector2d(37, 17))
                        .afterTime(0, verticalSlides.slidesUp(SLIDES_GRAB))
                        .waitSeconds(WAIT_TO_ALIGN)
                        // wait for human player to align
                        .strafeTo(new Vector2d(37, 4))
                        .waitSeconds(WAIT_TO_GRAB)
                        // go to specimen
                        .stopAndAdd(claw.clawClose())
                        .stopAndAdd(verticalSlides.slidesUp(SLIDES_UP+50))
                        // turn and line up with submersible
                        .strafeToSplineHeading(new Vector2d(5, 20), Math.toRadians(-90))
                        // put specimen on bar
                        .strafeTo(new Vector2d(5, 32))
                        .stopAndAdd(claw.clawOpen())



                        // THIRD

                        .strafeTo(new Vector2d(5, 20))
                        .afterTime(0, verticalSlides.slidesDown(0))
                        .strafeToSplineHeading(new Vector2d(37, 17), Math.toRadians(90))
                        .afterTime(0, verticalSlides.slidesUp(SLIDES_GRAB))
                        .waitSeconds(WAIT_TO_ALIGN)
                        // wait for human player to align
                        .strafeTo(new Vector2d(37, 4))
                        .waitSeconds(WAIT_TO_GRAB)
                        // go to specimen
                        .stopAndAdd(claw.clawClose())
                        .stopAndAdd(verticalSlides.slidesUp(SLIDES_UP+50))
                        // turn and line up with submersible
                        .strafeToSplineHeading(new Vector2d(5, 20), Math.toRadians(-90))
                        // put specimen on bar
                        .strafeTo(new Vector2d(5, 32))
                        .stopAndAdd(claw.clawOpen())
                        .afterTime(0, claw.clawClose())
                        .strafeTo(new Vector2d(5, 10))
                        // cleanup
                        .stopAndAdd(outtakeFlip.outtakeFlipIn())
                        .waitSeconds(0.5)
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
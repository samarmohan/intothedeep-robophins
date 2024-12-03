/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOP", group="Linear OpMode")
public class TeleOP extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        DcMotor rhinoHorn = hardwareMap.get(DcMotor.class, "Rhino Horn");

        // wheels
        ElapsedTime runtime = new ElapsedTime();
        DcMotor FL = hardwareMap.get(DcMotor.class, "Front Left Wheel");
        DcMotor BL = hardwareMap.get(DcMotor.class, "Back Left Wheel");
        DcMotor FR = hardwareMap.get(DcMotor.class, "Front Right Wheel");
        DcMotor BR = hardwareMap.get(DcMotor.class, "Back Right Wheel");

        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // slides
        DcMotor LS = hardwareMap.get(DcMotor.class, "Left Slide");
        DcMotor RS = hardwareMap.get(DcMotor.class, "Right Slide");
        DcMotor IS = hardwareMap.get(DcMotor.class, "Intake Slide");

        LS.setDirection(DcMotorSimple.Direction.REVERSE);

        LS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // outtake
        Servo outtakeClaw = hardwareMap.get(Servo.class, "Outtake Grasp");
        Servo outtakeFlip = hardwareMap.get(Servo.class, "Outtake Flip");

        // intake
        Servo intakeSwivel = hardwareMap.get(Servo.class, "Intake Swivel");
        Servo intakeGrasp = hardwareMap.get(Servo.class, "Intake Grasp");
        Servo intakeFlipRight = hardwareMap.get(Servo.class, "Intake Flip Right");
        Servo intakeFlipLeft = hardwareMap.get(Servo.class, "Intake Flip Left");

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();
        runtime.reset();
        // toggles so they don't reset every tick
        boolean clawOpen = true;
        boolean flipIn = true;
        boolean jointUp = true;
        boolean jointStart = true;
        double intakeState = 0;
        double jointPos = 0.8;
        double intakePower = 0;
        double graspPos = 1;
        double swivelPos = 0;

        double timeSinceClawToggle = runtime.seconds();
        double timeSinceFlipToggle = runtime.seconds();
        double timeSinceJointToggle = runtime.seconds();
        double timeSinceIntakeSwitch = runtime.seconds();
        while (opModeIsActive()) {
            //wheels
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            //ONLY FOR FIELD CENTRIC
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            //rotX = rotX * 1.1;  // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double multiplier = Math.max(gamepad1.left_trigger, 0.3);

            double flPower = multiplier * ((y + x + rx) / denominator);
            double blPower = multiplier * ((y - x + rx) / denominator);
            double frPower = multiplier * ((y - x - rx) / denominator);
            double brPower = multiplier * ((y + x - rx) / denominator);

            FL.setPower(flPower);
            BL.setPower(blPower);
            FR.setPower(frPower);
            BR.setPower(brPower);

            // slides
            int maxSlidePos = -4350;
            int maxIntakePos = -1500;
            double slidePower = gamepad1.right_stick_y;
            double intakeSlidePower = gamepad2.left_stick_y;

            int lsPosition = LS.getCurrentPosition();
            int rsPosition = RS.getCurrentPosition();
            int isPosition = BR.getCurrentPosition();

            if ((isPosition < maxIntakePos) && intakeSlidePower <= 0) {
                IS.setPower(0);

            } else {
                IS.setPower(intakeSlidePower);
            }

            if (((lsPosition < maxSlidePos) || (rsPosition < maxSlidePos)) && slidePower >= 0) {
                LS.setPower(0.1);
                RS.setPower(0.1);

            } else {
                LS.setPower(slidePower);
                RS.setPower(slidePower);
            }

            // outtake
            double clawPos;
            double flipPos;

            if (gamepad1.a && runtime.seconds() - timeSinceClawToggle > 0.5) {
                clawOpen = !clawOpen;
                timeSinceClawToggle = runtime.seconds();
            }
            if (clawOpen) {
                clawPos = 0.3;
            } else {
                clawPos = 0.4;
            }

            if (gamepad2.b && runtime.seconds() - timeSinceFlipToggle > 0.5) {
                flipIn = !flipIn;
                timeSinceFlipToggle = runtime.seconds();
            }
            if (flipIn) {
                flipPos = 1;
            } else {
                flipPos = 0;
            }

            outtakeClaw.setPosition(clawPos);
            outtakeFlip.setPosition(flipPos);

            // intake

            if (clawOpen) {
                graspPos = 0.8;
            } else {
                graspPos = 0.9;
            }

            swivelPos = 0.3;

            if (gamepad2.x && runtime.seconds() - timeSinceJointToggle > 0.5) {
                jointUp = !jointUp;
                jointStart = false;
                timeSinceJointToggle = runtime.seconds();
            }

            if (jointUp && jointStart){
                jointPos = 0.8;
            } else if (jointUp) {
                jointPos = 0.92;
            } else {
                jointPos = 0.15;
            }

            intakeSwivel.setPosition(swivelPos);
            intakeGrasp.setPosition(graspPos);
            intakeFlipRight.setPosition(jointPos);
            intakeFlipLeft.setPosition(1-jointPos);

            //rhino arm (hang)
            if (gamepad1.x) {
                rhinoHorn.setPower(1);
            } else if (gamepad1.y) {
                rhinoHorn.setPower(-0.5);
            } else {
                rhinoHorn.setPower(0);
            }

            // telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("fl power", flPower);
            telemetry.addData("bl power", blPower);
            telemetry.addData("fr power", frPower);
            telemetry.addData("br power", brPower);
            telemetry.addData("slide power", slidePower);
            telemetry.addData("intake slide power", intakeSlidePower);
            telemetry.addData("ls pos", lsPosition);
            telemetry.addData("rs pos", rsPosition);
            telemetry.addData("is pos", isPosition);
            telemetry.addData("joint pos", jointPos);
            telemetry.addData("claw pos", clawPos);
            telemetry.addData("flip pos", flipPos);
            telemetry.addData("intake POWER", intakePower);
            telemetry.addData("claw open", clawOpen);
            telemetry.update();
        }
    }
}
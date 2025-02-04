package com.ftcrobotcontroller.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(7.5, -62, Math.toRadians(90) ))
                // go to first sample
                .splineTo(new Vector2d(38, -13), Math.toRadians(90))
                // push all 3 back to human player station
                .strafeTo(new Vector2d(47, -13))
                .strafeTo(new Vector2d(47, -53))
                .strafeTo(new Vector2d(47, -13))
                .strafeTo(new Vector2d(56, -13))
                .strafeTo(new Vector2d(56, -53))
                .strafeTo(new Vector2d(56, -13))
                .strafeTo(new Vector2d(65, -13))
                .strafeTo(new Vector2d(65, -53))
                // go to clip first sample and hang it
                .splineTo(new Vector2d(40, -59), Math.toRadians(270))
                .waitSeconds(2)
                .splineTo(new Vector2d(4, -45), Math.toRadians(270))
                .strafeTo(new Vector2d(4, -32))
                .waitSeconds(1)
                // return back for second sample, same thing
                .splineTo(new Vector2d(40, -59), Math.toRadians(90))
                .waitSeconds(2)
                .splineTo(new Vector2d(4, -45), Math.toRadians(270))
                .strafeTo(new Vector2d(4, -32))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
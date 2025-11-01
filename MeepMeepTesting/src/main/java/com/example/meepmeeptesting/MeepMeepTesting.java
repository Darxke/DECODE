package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d; // NOTE: no ".geometry" in 1.0
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep mm = new MeepMeep(800);

        RoadRunnerBotEntity bot = new DefaultBotBuilder(mm)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        bot.runAction(bot.getDrive().actionBuilder(new Pose2d(56, -16, Math.toRadians(210)))
                .strafeToConstantHeading(new Vector2d(56, -56))
                .strafeToConstantHeading(new Vector2d(56,-16))

//                .splineToLinearHeading(new Pose2d(12.5, -38, Math.toRadians(270)), Math.toRadians(270))
//                .lineToY(-52)
//                .strafeToLinearHeading(new Vector2d(-15, -15), Math.toRadians(225))



                .build());

        mm.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true).setBackgroundAlpha(0.95f)
                .addEntity(bot).start();
    }
}

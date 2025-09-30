package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class FarRedAuto {

    //TODO ONCE NOT IN MEEP MEEP MAKE THE MOTIF = WHATEVER THE APRIL TAG DISCOVERS AND USE THAT
    public static String motif = "GPP";

    //Location for the robot to start
    //TODO Change if starting close/far and for red/blue
    static double startX = 56;
    static double startY = -8;
    static double startHeading = -225;
    static Pose2d startPose = new Pose2d(startX,startY,Math.toRadians(startHeading));

    //Position and heading the robot needs to be to launch the artifact
    //TODO Find where bc right now the position is a complete guess
    static double launchX = -16;
    static double launchY = 0;
    static double launchHeading = Math.toRadians(180); //In degrees
    static Vector2d launchPose = new Vector2d(launchX,launchY);

    public static void main(String[] args) {
        switch(motif) {
            case "GPP":
                GPP();
                break;
            case "PGP":
                PGP();
                break;
            case "PPG":
                PPG();
                break;
        }
    }

    public static void GPP(){
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(startPose)
                .waitSeconds(0)
                .splineTo(launchPose,Math.toRadians(315))
                //Scan for which artifacts and shoot (waitSeconds is placeholder for shooting)
                .waitSeconds(4)
                .strafeToLinearHeading(new Vector2d(-12,32),Math.toRadians(90))
                //This is where we pick up 3 new specemin and sort maybe?
                .strafeToLinearHeading(new Vector2d(-12,52), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-12,40), Math.toRadians(90))
                .splineTo(launchPose,Math.toRadians(135))
                .waitSeconds(4)
                .strafeToLinearHeading(new Vector2d(12,32),Math.toRadians(90))
                //This is where we pick up 3 new specemin and sort maybe?
                .strafeToLinearHeading(new Vector2d(12,52), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(12,40), Math.toRadians(90))
                .splineTo(launchPose,Math.toRadians(135))
                .waitSeconds(4)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    public static void PGP(){
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(startPose)
                .waitSeconds(0)
                .splineTo(launchPose,Math.toRadians(315))
                //Scan for which artifacts and shoot (waitSeconds is placeholder for shooting)
                .waitSeconds(4)
                .strafeToLinearHeading(new Vector2d(-12,32),Math.toRadians(90))
                //This is where we pick up 3 new specemin and sort maybe?
                .strafeToLinearHeading(new Vector2d(-12,52), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-12,40), Math.toRadians(90))
                .splineTo(launchPose,Math.toRadians(135))
                .waitSeconds(4)
                .strafeToLinearHeading(new Vector2d(12,32),Math.toRadians(90))
                //This is where we pick up 3 new specemin and sort maybe?
                .strafeToLinearHeading(new Vector2d(12,52), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(12,40), Math.toRadians(90))
                .splineTo(launchPose,Math.toRadians(135))
                .waitSeconds(4)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    public static void PPG(){
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(startPose)
                .waitSeconds(0)
                .splineTo(launchPose,Math.toRadians(315))
                //Scan for which artifacts and shoot (waitSeconds is placeholder for shooting)
                .waitSeconds(4)
                .strafeToLinearHeading(new Vector2d(-12,32),Math.toRadians(90))
                //This is where we pick up 3 new specemin and sort maybe?
                .strafeToLinearHeading(new Vector2d(-12,52), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-12,40), Math.toRadians(90))
                .splineTo(launchPose,Math.toRadians(135))
                .waitSeconds(4)
                .strafeToLinearHeading(new Vector2d(12,32),Math.toRadians(90))
                //This is where we pick up 3 new specemin and sort maybe?
                .strafeToLinearHeading(new Vector2d(12,52), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(12,40), Math.toRadians(90))
                .splineTo(launchPose,Math.toRadians(135))
                .waitSeconds(4)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }


}
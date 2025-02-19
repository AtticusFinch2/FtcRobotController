package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPosebdc = new Pose2d(16, 64, Math.toRadians(270));
        Pose2d startPoserdc = new Pose2d(16, -64, Math.toRadians(90));
        //IMPORTANT: COMMENT OUT BOT ENTITIES DEPENDING ON WHICH PATHING WE WANT TO TEST

        RoadRunnerBotEntity BlueDoubleCloseSpike1 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(166), Math.toRadians(60), 13)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPosebdc)
                                .forward(33)
                                .strafeLeft(3)
                                .strafeRight(3)
                                .back(12)
                                .lineToLinearHeading(new Pose2d(38, 40, Math.toRadians(180)))
                                .build()
                );

        RoadRunnerBotEntity BlueDoubleCloseSpike2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(166), Math.toRadians(60), 13)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPosebdc)
                                .splineTo(new Vector2d(14, 30), Math.toRadians(180))
                                .strafeLeft(6)
                                .strafeRight(6)
                                .lineToLinearHeading(new Pose2d(38, 40, Math.toRadians(180)))
                                .build()
                );

        RoadRunnerBotEntity BlueDoubleCloseSpike3 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(166), Math.toRadians(60), 13)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPosebdc)
                                .forward(5)
                                .strafeLeft(14)
                                .splineTo(new Vector2d(15, 38), Math.toRadians(180))
                                .strafeLeft(5)
                                .strafeRight(10)
                                .lineToLinearHeading(new Pose2d(38, 36, Math.toRadians(180)))
                                .build()
                );

        RoadRunnerBotEntity RedDoubleCloseSpike1 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(166), Math.toRadians(60), 13)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(40,40,Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(-5, 40, Math.toRadians(180)))

                                .build()
                );

        RoadRunnerBotEntity RedDoubleCloseSpike2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(166), Math.toRadians(60), 13)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPoserdc)
                                .splineTo(new Vector2d(16, -18), Math.toRadians(180))
                                .strafeRight(5)
                                .back(5)
                                .lineToLinearHeading(new Pose2d (38, -40, Math.toRadians(180)))
                                .build()
                );

        RoadRunnerBotEntity RedDoubleCloseSpike3 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(166), Math.toRadians(60), 13)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPoserdc)
                                .forward(30)
                                .strafeRight(14)
                                .lineToLinearHeading(new Pose2d (38, -40, Math.toRadians(180)))
                                .build()
                );


        // MEEP MEEP SIMULATOR RUNNER
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(RedDoubleCloseSpike1)
                .start();
    }
}
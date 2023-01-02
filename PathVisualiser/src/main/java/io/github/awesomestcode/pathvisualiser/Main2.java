package io.github.awesomestcode.pathvisualiser;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Main2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800, 50);
        meepMeep.setAxesInterval(12);

        Pose2d startPose = new Pose2d(36, -62.5, Math.toRadians(90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, 3.8603167394869375, Math.toRadians(60), 14.4)
                .setStartPose(startPose)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setDimensions(13, 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)

                                // First Junction
                                .forward(20 + 12)
                                .back(12)
                                .forward(0.01)
                                .splineTo(new Vector2d(12 - 9, -36 + 9), Math.toRadians(130))
                                .waitSeconds(0.5)

                                // Go to origin
                                .setReversed(true)
                                .splineTo(new Vector2d(36, -62.5 + 22), Math.toRadians(-90))
                                .setReversed(false)
                                .forward(6)
                                .waitSeconds(0.5)
                                .build()
                );

        MeepMeep thing = meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_LIGHT)
                .setAxesInterval(12)
                .setDarkMode(false)
                .setBackgroundAlpha(1.0f)
                .addEntity(myBot)
                .setAxesInterval(12);

        thing.getWindowFrame().setTitle("Delbotics Path Visualisation");
        thing.start();
        thing.setAxesInterval(12);
    }
}
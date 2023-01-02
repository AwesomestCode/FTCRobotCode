package io.github.awesomestcode.pathvisualiser;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Main {
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
                                .splineTo(new Vector2d(12 - 8, -36 + 8), Math.toRadians(130))
                                .waitSeconds(0.5)

                                // Go to stack
                                .back(8)
                                .lineToLinearHeading(new Pose2d(16, -14, Math.toRadians(30)))
                                .splineTo(new Vector2d(60, -12), Math.toRadians(0))
                                //.lineToSplineHeading(new Pose2d(12, -30, Math.toRadians(0)))
                                //.splineTo(new Vector2d(60, -12), Math.toRadians(-90))
                                .waitSeconds(0.5)

                                // Deposit
                                .lineToLinearHeading(new Pose2d(40, -12, Math.toRadians(180)))
                                .splineTo(new Vector2d(36 - 8, -12 + 8), Math.toRadians(135))
                                .waitSeconds(0.5)
                                //.splineToLinearHeading(new Pose2d(24, -12, Math.toRadians(90)), Math.toRadians(90))

                                // Go Back to Stack
                                .splineToSplineHeading(new Pose2d(40, -12, Math.toRadians(0)), Math.toRadians(0))
                                .lineToLinearHeading(new Pose2d(60, -12, Math.toRadians(0)))
                                .waitSeconds(0.5)

                                // Deposit
                                .lineToLinearHeading(new Pose2d(40, -12, Math.toRadians(180)))
                                .splineTo(new Vector2d(36 - 8, -12 + 8), Math.toRadians(135))
                                .waitSeconds(0.5)

                                // Go Back to Stack
                                .splineToSplineHeading(new Pose2d(40, -12, Math.toRadians(0)), Math.toRadians(0))
                                .lineToLinearHeading(new Pose2d(60, -12, Math.toRadians(0)))
                                .waitSeconds(0.5)

                                // Deposit
                                .lineToLinearHeading(new Pose2d(40, -12, Math.toRadians(180)))
                                .splineTo(new Vector2d(36 - 8, -12 + 8), Math.toRadians(135))
                                .waitSeconds(0.5)

                                // Park
                                .setReversed(true)
                                .splineTo(new Vector2d(36, -36), Math.toRadians(-90))
                                .waitSeconds(0.2)
                                .setReversed(false)
                                .strafeLeft(24)

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
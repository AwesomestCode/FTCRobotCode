package io.github.awesomestcode.pathvisualiser;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.Arrays;

public class AllRightTesting {
    static final boolean RIGHT = false;
    public static void main(String[] args) throws InterruptedException {
        MeepMeep meepMeep = new MeepMeep(800, 50);
        meepMeep.setAxesInterval(12);

        Pose2d startPose = new Pose2d(RIGHT ? 36 : -36, -62.5, Math.toRadians(90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(55, 50, 3.8603167394869375, Math.toRadians(180), 14.4)
                .setStartPose(startPose)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setDimensions(13.5, 14.5)
                .build();


        TrajectorySequence mainTraj = myBot.getDrive().trajectorySequenceBuilder(startPose)
                //TrajectorySequence clearSignal = myBot.getDrive().trajectorySequenceBuilder(startPose)
                .setConstraints(new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(3.8603167394869375),
                        new MecanumVelocityConstraint(55, 14.4)
                )), new ProfileAccelerationConstraint(50))
                .lineTo(new Vector2d(RIGHT ? 36 : -36, -16))
                .setConstraints(new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(3.8603167394869375),
                        new MecanumVelocityConstraint(10, 14.4)
                )), new ProfileAccelerationConstraint(10))
                .lineTo(new Vector2d(RIGHT ? 36 : -36, -8))
                .setConstraints(new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(3.8603167394869375),
                        new MecanumVelocityConstraint(60, 14.4)
                )), new ProfileAccelerationConstraint(60))
                .back(12)
                .forward(0.01)
                .resetConstraints()
                //        .build();

                //TrajectorySequence getToJunction = myBot.getDrive().trajectorySequenceBuilder(clearSignal.end())
                .splineTo(new Vector2d(RIGHT ? (36 - 9) : -(36 - 9), -12 + 9), Math.toRadians(RIGHT ? 135 : 45))
                .waitSeconds(1.5) // this is about the time it takes to simulate a deposit
                //        .build();

                //TrajectorySequence returnToOrigin = myBot.getDrive().trajectorySequenceBuilder(getToJunction.end())
                /*.setReversed(true)
                .splineTo(new Vector2d(36, -62.5 + 22), Math.toRadians(-90))
                .setReversed(false)
                .forward(6)
                .waitSeconds(0.5)*/
                //        .build();

                //TrajectorySequence goToStack1 = myBot.getDrive().trajectorySequenceBuilder(getToJunction.end())
                .setReversed(true)
                //.lineToSplineHeading(new Pose2d(12 - 6, -36 + 6, Math.toRadians(120)))
                //.lineToSplineHeading(new Pose2d(10, -35, Math.toRadians(90)))
                //.lineToConstantHeading(new Vector2d(RIGHT ? (36 - 3) : -(36 - 3), -12 + 3))
                //.splineTo(new Vector2d(RIGHT ? 10 : -10, -36), Math.toRadians(-90))
                //.setReversed(false)
                //.splineTo(new Vector2d(RIGHT ? 31 : -31, -12), Math.toRadians(RIGHT ? 0 : 180))
                .splineToSplineHeading(new Pose2d(RIGHT ? 55 : -55, -12, Math.toRadians(RIGHT ? 0 : 180)), Math.toRadians(RIGHT ? 0 : 180))
                //.splineTo(new Vector2d(58, -12), Math.toRadians(180))
                //.lineTo(new Vector2d(64, -12))
                //.lineToSplineHeading(new Pose2d(12, -30, Math.toRadians(0)))
                //.splineTo(new Vector2d(60, -12), Math.toRadians(-90))
                .waitSeconds(1.5) // simulate the alignment
                //        .build();

                // TrajectorySequence goForwardToStack = drive.trajectorySequenceBuilder(goToStack1.end())
                .lineTo(new Vector2d(RIGHT ? 68 : -68, -12))
                .waitSeconds(2.5) // simulate the alignment and pickup
                //.build();

                //TrajectorySequence goToJunction2 = myBot.getDrive().trajectorySequenceBuilder(goToStack1.end())
                .back(6)
                //.lineToSplineHeading(new Pose2d(40, -12, Math.toRadians(180)))
                .splineToSplineHeading(new Pose2d(RIGHT ? 36 - 9 : -(36 - 9), -12 + 9, Math.toRadians(RIGHT ? 135 : 45)), Math.toRadians(RIGHT ? 135 : 45))
                .waitSeconds(1.5) //simulate the deposit
                //        .build();

                //TrajectorySequence Left Park
                /*.setReversed(true)
                .back(6)
                .splineTo(new Vector2d(24, -14), Math.toRadians(180))
                .lineTo(new Vector2d(12, -14))
                .setReversed(false)*/
                //TrajectorySequenceCentrePark
                /*.setReversed(true)
                .splineTo(new Vector2d(36, -14), Math.toRadians(180))
                .setReversed(false)*/
                //TrajectorySequenceRightPark
                .setReversed(true)
                .splineTo(new Vector2d(RIGHT ? 50 : -50, -14), Math.toRadians(RIGHT ? 180 : 0))
                .setReversed(false)
                .forward(10)


                //TrajectorySequence goLeft = myBot.getDrive().trajectorySequenceBuilder(returnToOrigin.end())
                //.strafeLeft(24)
                //        .build();

                //TrajectorySequence goRight = myBot.getDrive().trajectorySequenceBuilder(returnToOrigin.end())
                //.strafeRight(24)
                .build();

        //Create a new trajectory sequence that combines all of the above
        //TrajectorySequence trajectorySequence = myBot.getDrive().trajectorySequenceBuilder(startPose)


        myBot.followTrajectorySequence(mainTraj);

        MeepMeep thing = meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_LIGHT)
                .setAxesInterval(12)
                .setDarkMode(false)
                .setBackgroundAlpha(1.0f)
                .addEntity(myBot)
                .setAxesInterval(12);

        thing.getWindowFrame().setTitle("Delbotics Path Visualisation");
        thing.start();
        //sleep(250);
        //myBot.followTrajectorySequence(returnToOrigin);

        thing.setAxesInterval(12);
    }
}

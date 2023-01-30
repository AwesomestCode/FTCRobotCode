package org.firstinspires.ftc.teamcode.autotools;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import org.firstinspires.ftc.teamcode.auto.AutoConstants;
import org.firstinspires.ftc.teamcode.auto.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.auto.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.SlidePositionSetter;
import org.firstinspires.ftc.teamcode.subsystems.SlidePositions;

import java.util.Arrays;

public final class TrajectoryGenerator {

    private static TrajectoryGenerator generator;

    public static TrajectoryGenerator getInstance() {
        if (generator == null) {
            generator = new TrajectoryGenerator();
        }
        return generator;
    }

    public enum Alliance {
        RED,
        BLUE
    }
    public enum Side {
        LEFT,
        RIGHT
    }

    private Alliance alliance;
    private Side side;

    private SampleMecanumDrive drive;

    boolean initialised;

    private final Pose2d startPose = new Pose2d(36, -62.5, Math.toRadians(90));

    public Pose2d getStartPose() {
        return startPose;
    }

    private final Pose2d atConePose = new Pose2d(64, -12, Math.toRadians(0));

    private TrajectorySequence clearSignal;

    private TrajectorySequence getToJunction;

    private TrajectorySequence goToStack1;

    private TrajectorySequence goForwardToStack;


    private TrajectorySequence goToJunction2;

    private TrajectorySequence parkLeft;

    private TrajectorySequence parkCentre;

    private TrajectorySequence parkRight;

    public TrajectorySequence getClearSignal() {
        if(!initialised) {
            throw new IllegalStateException("TrajectoryGenerator not initialised");
        }
        return clearSignal;
    }

    public TrajectorySequence getGoToJunction1() {
        if(!initialised) {
            throw new IllegalStateException("TrajectoryGenerator not initialised");
        }
        return getToJunction;
    }

    public TrajectorySequence getGoToStack1() {
        if(!initialised) {
            throw new IllegalStateException("TrajectoryGenerator not initialised");
        }
        return goToStack1;
    }

    public TrajectorySequence getGoForwardToStack() {
        if(!initialised) {
            throw new IllegalStateException("TrajectoryGenerator not initialised");
        }
        return goForwardToStack;
    }

    public TrajectorySequence getGoToJunction2() {
        if(!initialised) {
            throw new IllegalStateException("TrajectoryGenerator not initialised");
        }
        return goToJunction2;
    }

    public TrajectorySequence getParkLeft() {
        if(!initialised) {
            throw new IllegalStateException("TrajectoryGenerator not initialised");
        }
        return parkLeft;
    }

    public TrajectorySequence getParkCentre() {
        if(!initialised) {
            throw new IllegalStateException("TrajectoryGenerator not initialised");
        }
        return parkCentre;
    }

    public TrajectorySequence getParkRight() {
        if(!initialised) {
            throw new IllegalStateException("TrajectoryGenerator not initialised");
        }
        return parkRight;
    }

    public Pose2d getAtConePose() {
        return atConePose;
    }

    public void initialise(Alliance alliance, Side side, SampleMecanumDrive drive, SlidePositionSetter slideSystem) {
        if(alliance == null || side == null) {
            throw new IllegalArgumentException("Alliance and side must be non-null");
        }
        if(this.alliance ==  alliance && this.side == side && drive.equals(this.drive)) {
            System.out.println("TrajectoryGenerator already initialised with the same alliance and side");
            return; //silently exit since we already have the correct values
        }

        clearSignal = drive.trajectorySequenceBuilder(startPose)
                .setConstraints(new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(3.8603167394869375),
                        new MecanumVelocityConstraint(30, 14.4)
                )), new ProfileAccelerationConstraint(25))
                .forward(20 + 12)
                .setConstraints(new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(3.8603167394869375),
                        new MecanumVelocityConstraint(50, 14.4)
                )), new ProfileAccelerationConstraint(45))
                .back(12)
                .forward(0.01)
                .resetConstraints()
                .build();

        getToJunction = drive.trajectorySequenceBuilder(clearSignal.end())
                .splineTo(new Vector2d(12 - 9, -36 + 9), Math.toRadians(AutoConstants.FIRST_JUNCTION_ROT))
                .waitSeconds(0.25)
                .build();

        goToStack1 = drive.trajectorySequenceBuilder(getToJunction.end())
                .addTemporalMarker(1, () -> slideSystem.setPosition(SlidePositions.WALL.getPosition()))
                .setReversed(true)
                .lineToConstantHeading(new Vector2d(12 - 3, -36 + 3))
                .splineTo(new Vector2d(10, -36), Math.toRadians(-90))
                .setReversed(false)
                .splineTo(new Vector2d(31, -10.5), Math.toRadians(0))
                .splineTo(new Vector2d(55, -10.5), Math.toRadians(0))
                .build();

        goForwardToStack = drive.trajectorySequenceBuilder(goToStack1.end())
                .lineTo(new Vector2d(68, -10.5))
                .build();

        goToJunction2 = drive.trajectorySequenceBuilder(atConePose)
                .back(6)
                //.lineToSplineHeading(new Pose2d(40, -12, Math.toRadians(180)))
                .addSpatialMarker(new Vector2d(48, -12), () -> slideSystem.setPosition(SlidePositions.TOP.getPosition()))
                .splineToSplineHeading(new Pose2d(36 - 9, -12 + 9, Math.toRadians(135)), Math.toRadians(135))
                .waitSeconds(0.5)
                .build();

        parkLeft = drive.trajectorySequenceBuilder(goToJunction2.end())
                .addTemporalMarker(1, () -> slideSystem.setPosition(SlidePositions.WALL.getPosition()))
                .setConstraints(new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(3.0),
                        new MecanumVelocityConstraint(50, 14.4)
                )), new ProfileAccelerationConstraint(45))
                .setReversed(true)
                .back(6)
                .splineTo(new Vector2d(24, -14), Math.toRadians(180))
                .lineTo(new Vector2d(12, -14))
                .setReversed(false)
                .resetConstraints()
                .build();

        parkCentre = drive.trajectorySequenceBuilder(goToJunction2.end())
                .addTemporalMarker(2, () -> slideSystem.setPosition(SlidePositions.WALL.getPosition()))
                .setConstraints(new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(3.0),
                        new MecanumVelocityConstraint(50, 14.4)
                )), new ProfileAccelerationConstraint(45))
                .setReversed(true)
                .splineTo(new Vector2d(36, -14), Math.toRadians(180))
                .setReversed(false)
                .resetConstraints()
                .build();

        parkRight = drive.trajectorySequenceBuilder(goToJunction2.end())
                .setConstraints(new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(3.0),
                        new MecanumVelocityConstraint(50, 14.4)
                )), new ProfileAccelerationConstraint(45))
                .setReversed(true)
                .splineTo(new Vector2d(50, -14), Math.toRadians(180))
                .setReversed(false)
                .forward(10)
                .resetConstraints()
                .build();

        initialised = true;
    }
}

package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.auto.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.auto.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.libauto.DetectAprilTagZoneUtil;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivetrainMixer;
import org.firstinspires.ftc.teamcode.subsystems.SlidePositions;

@Autonomous
@Config
public class TripleAsteroid extends LinearOpMode {
    public static int ZONE = 3;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.speak("Stand back comma will do.");

        waitForStart();

        telemetry.speak("I will commence operations.");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to start the bot at x: 10, y: -8, heading: 90 degree
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        ZONE = DetectAprilTagZoneUtil.getZone(hardwareMap, telemetry);

        telemetry.speak("I have detected the zone as " + ZONE);

        TrajectorySequence goForward = drive.trajectorySequenceBuilder(startPose)
                        .forward(30)
                        .waitSeconds(0.5)
                        .build();

        TrajectorySequence goLeft = drive.trajectorySequenceBuilder(goForward.end())
                        .strafeLeft(24)
                        .build();

        TrajectorySequence goRight = drive.trajectorySequenceBuilder(goForward.end())
                .strafeRight(24)
                .build();

        drive.followTrajectorySequence(goForward);
        if(ZONE == 1) {
            drive.followTrajectorySequence(goLeft);
        } else if(ZONE == 3) {
            drive.followTrajectorySequence(goRight);
        }

        telemetry.speak("I have finished parking. Killing.");
        sleep(1000);
    }
}

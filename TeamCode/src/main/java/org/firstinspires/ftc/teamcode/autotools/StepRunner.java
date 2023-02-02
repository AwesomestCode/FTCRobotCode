package org.firstinspires.ftc.teamcode.autotools;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.AutoConstants;
import org.firstinspires.ftc.teamcode.auto.PoseData;
import org.firstinspires.ftc.teamcode.auto.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.auto.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.JunctionPositionSensor;
import org.firstinspires.ftc.teamcode.subsystems.SlidePositionSetter;
import org.firstinspires.ftc.teamcode.subsystems.SlidePositions;
import org.firstinspires.ftc.teamcode.subsystems.TapePositionSensor;

public class StepRunner {

    private static StepRunner runner;
    private Telemetry telemetry;
    private SampleMecanumDrive drive;
    private TrajectoryGenerator.Side side;

    private StepRunner() {}

    public static StepRunner getInstance(Telemetry telemetry, SampleMecanumDrive drive, TrajectoryGenerator.Side side) {
        if (runner == null) {
            runner = new StepRunner();
        }
        runner.telemetry = telemetry;
        runner.drive = drive;
        runner.side = side;
        return runner;
    }

    private void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void rotate(double rotation) {
        drive.setWeightedDrivePower(new Pose2d(0, 0, rotation));
    }

    public void parkFromJunction2(TrajectorySequence parkLeft, TrajectorySequence parkCentre, TrajectorySequence parkRight, int zone, SlidePositionSetter slideSystem) {
        if(zone == 1) {
            telemetry.speak("I am parking in the left zone");
            telemetry.clearAll();
            telemetry.addLine("Parking in left zone");
            telemetry.update();
            drive.followTrajectorySequence(parkLeft);
        } else if(zone == 2) {
            telemetry.speak("I am parking in the centre zone");
            telemetry.clearAll();
            telemetry.addLine("Parking in centre zone");
            telemetry.update();
            drive.followTrajectorySequence(parkCentre);
        } else if(zone == 3) {
            telemetry.speak("I am parking in the right zone");
            telemetry.clearAll();
            telemetry.addLine("Parking in right zone");
            telemetry.update();
            drive.followTrajectorySequence(parkRight);
        } else {
            telemetry.speak("I have failed to detect the zone, so I am parking in the centre zone");
            telemetry.clearAll();
            telemetry.addLine("Parking in centre zone");
            telemetry.update();
            drive.followTrajectorySequence(parkCentre);
        }
        slideSystem.setPosition(0);
        PoseData.lastAngle = -(drive.getLocalizer().getPoseEstimate().getHeading() - 90);
    }

    public void goSecondJunctionFromStack(JunctionPositionSensor sensor, CRServoImplEx intake, SlidePositionSetter slideSystem, TrajectorySequence goToJunction2) {
        drive.followTrajectorySequence(goToJunction2);
        telemetry.clearAll();
        telemetry.addLine("At junction, attempting alignment");
        telemetry.update();
        sensor.align(this::rotate);

        drive.updatePoseEstimate();

        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(2)
                .build());

        telemetry.speak("Finished re-aligning");
        slideSystem.setPosition(SlidePositions.TOP.getPosition() - 200);
        sleep(250);
        intake.setPower(-0.3);
        sleep(1000);
        slideSystem.setPosition(SlidePositions.TOP.getPosition());
    }

    public void goStackJunction1(DigitalChannel led1, MultipleTelemetry multipleTelemetry, TapePositionSensor tapeSensor, CRServoImplEx intake, SlidePositionSetter slideSystem, Pose2d atConePose, TrajectorySequence goToStack1, TrajectorySequence goForwardToStack) {
        drive.followTrajectorySequence(goToStack1);

        telemetry.clearAll();
        telemetry.addLine("At stack, attempting to align");
        telemetry.update();

        telemetry.speak("Attempting to align");
        boolean spoken = false;

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while(!tapeSensor.isInRange() && timer.milliseconds() < 500) {
            if(!spoken) telemetry.speak("Sensor not in range");
            spoken = true;
            drive.setWeightedDrivePower(new Pose2d(0, this.side == TrajectoryGenerator.Side.RIGHT ? -0.4 : 0.4, 0));
            multipleTelemetry.update();
            //telemetry.update();
        }

        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

        telemetry.clearAll();
        telemetry.addLine("In Range");
        telemetry.update();

        if(spoken) {
            telemetry.speak("Now in range");
        } else {
            telemetry.speak("Already in range");
        }

        doAlignment(multipleTelemetry, tapeSensor, drive);
        telemetry.addLine("Alignment successful");
        telemetry.update();

        drive.setPoseEstimate(new Pose2d(this.side == TrajectoryGenerator.Side.RIGHT ? 55 : -55, -12));

        telemetry.addLine("Trying to get to stack");
        telemetry.update();

        drive.followTrajectorySequence(goForwardToStack);

        telemetry.speak("Attempting to align");
        spoken = false;

        timer.reset();

        while (!tapeSensor.isInRange() && timer.milliseconds() < 500) {
            if (!spoken) telemetry.speak("Sensor not in range");
            spoken = true;
            drive.setWeightedDrivePower(new Pose2d(0,  this.side == TrajectoryGenerator.Side.RIGHT ? -0.4 : 0.4, 0));
            multipleTelemetry.update();
            //telemetry.update();
        }

        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

        telemetry.clearAll();
        telemetry.addLine("In Range");
        telemetry.update();

        doAlignment(multipleTelemetry, tapeSensor, drive);

        slideSystem.setPosition(SlidePositions.WALL.getPosition() - 250);
        intake.setPower(0.3);
        sleep(750);
        slideSystem.setPosition(SlidePositions.WALL.getPosition() + 500);
        led1.setState(false);
        sleep(250);
        led1.setState(true);

        drive.setPoseEstimate(atConePose);
    }

    public void goFirstJunction(JunctionPositionSensor sensor, CRServoImplEx intake, SlidePositionSetter slideSystem, TrajectorySequence getToJunction) {
        drive.followTrajectorySequence(getToJunction);
        telemetry.clearAll();
        telemetry.addLine("At junction, attempting alignment");
        telemetry.update();
        sensor.align(this::rotate);
        drive.updatePoseEstimate();
        //drive.setPoseEstimate(getToJunction.end());
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(2)
                .build());
        telemetry.speak("Finished re-aligning");
        slideSystem.setPosition(SlidePositions.TOP.getPosition() - 200);
        sleep(250);
        intake.setPower(-1);
        sleep(500);
        intake.setPower(0);
        slideSystem.setPosition(SlidePositions.TOP.getPosition());
        intake.setPower(0.3);
        sleep(200);
        intake.setPower(0);
    }

    public void clearSignalFromOrigin(DigitalChannel led0, CRServoImplEx intake, SlidePositionSetter slideSystem, TrajectorySequence clearSignal) {
        intake.setPower(0.5);
        drive.followTrajectorySequence(clearSignal);
        intake.setPower(0.3);
        slideSystem.setPosition(SlidePositions.TOP.getPosition());
        led0.setState(true);
    }

    private void doAlignment(MultipleTelemetry multipleTelemetry, TapePositionSensor tapeSensor, SampleMecanumDrive drive) {
        boolean res = tapeSensor.align((double strafeAmount) -> {
            drive.setWeightedDrivePower(new Pose2d(0, strafeAmount * -AutoConstants.TAPE_STRAFE_COEFFICIENT, 0));
            multipleTelemetry.addData("Strafe", strafeAmount * AutoConstants.TAPE_STRAFE_COEFFICIENT);
            multipleTelemetry.update();
        }, multipleTelemetry);

        if(res) {
            telemetry.speak("Aligned successfully");
        } else {
            telemetry.speak("Failed to align");
        }


        telemetry.clearAll();
    }
}

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.auto.DetectAprilTagZoneUtil;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivetrainMixer;

@Autonomous
@Config
public class DoubleAsteroid extends LinearOpMode {
    public static int FORWARD_DISTANCE = 650;
    public static int STRAFE_DISTANCE = 600;

    public static int ZONE = 3;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx frontLeft = (DcMotorEx) hardwareMap.dcMotor.get("frontLeft");
       // DcMotorEx leftEncoder = frontLeft;
        DcMotorEx frontRight = (DcMotorEx) hardwareMap.dcMotor.get("frontRight");
        DcMotorEx rearLeft = (DcMotorEx) hardwareMap.dcMotor.get("rearLeft");
        DcMotorEx strafeEncoder = rearLeft;
        DcMotorEx rearRight = (DcMotorEx) hardwareMap.dcMotor.get("rearRight");
        DcMotorEx rightEncoder = rearRight;

        MecanumDrivetrainMixer mixer = new MecanumDrivetrainMixer(frontLeft, frontRight, rearLeft, rearRight);

        waitForStart();

        //int leftStart = leftEncoder.getCurrentPosition();
        int rightStart = -rightEncoder.getCurrentPosition();
        int strafeStart = strafeEncoder.getCurrentPosition();


        double strafe;
        double left;
        double right;

        int stage = -1;
        // STAGE 0 is the first stage, where the robot is driving forward
        // STAGE 1 is the second stage, where the robot is strafing

        while(opModeIsActive()) {
            strafe = ((double) (strafeStart - strafeEncoder.getCurrentPosition()) / 8192) * 35 * Math.PI;
            //left = ((double) (leftStart - leftEncoder.getCurrentPosition()) / 8192) * 35 * Math.PI; // the left encoder is broken so let's ignore it
            right = ((double) (rightStart - -rightEncoder.getCurrentPosition()) / 8192) * 35 * Math.PI;

            //mixer.setMovement( ((500 - strafe)/500.0) * 0.2d, ((500 - right)/500.0) * 0.5d, 0.0);

            telemetry.addData("Stage", stage);
            if (stage == -1) {
                ZONE = DetectAprilTagZoneUtil.getZone(hardwareMap, telemetry);
                stage++;
            } else if (stage == 0) {
                mixer.setMovement(((FORWARD_DISTANCE - strafe) / FORWARD_DISTANCE) * 0.0d, Math.min(0.5, Math.max(((FORWARD_DISTANCE - right) / FORWARD_DISTANCE) * 0.5d, 0.1d)), 0.0);
                telemetry.addData("Distance from Target", Math.abs(FORWARD_DISTANCE - right));
                if (Math.abs(FORWARD_DISTANCE - right) < 50) { //if we're within 5 cm of target
                    //leftStart = leftEncoder.getCurrentPosition();
                    rightStart = -rightEncoder.getCurrentPosition();
                    strafeStart = strafeEncoder.getCurrentPosition();
                    mixer.setMovement(0, 0, 0);
                    sleep(100);
                    stage++;
                }
            } else if (stage == 1) {
                if(ZONE == 1) {
                    mixer.setMovement(-Math.min(0.5, Math.max(((STRAFE_DISTANCE - (-strafe)) / STRAFE_DISTANCE) * 0.5d, 0.2d)), ((STRAFE_DISTANCE - right) / STRAFE_DISTANCE) * 0.0d, 0.0);
                    telemetry.addData("Distance from Target", STRAFE_DISTANCE - (-strafe));
                    if (Math.abs(Math.abs(STRAFE_DISTANCE) - (-strafe)) < 100) { //if we're within 10 cm of target
                        stage++;
                        //leftStart = leftEncoder.getCurrentPosition();
                        rightStart = -rightEncoder.getCurrentPosition();
                        strafeStart = strafeEncoder.getCurrentPosition();
                    }
                }
                if(ZONE == 2) { //skip the stage because we do not need strafe
                    stage++;
                    //leftStart = leftEncoder.getCurrentPosition();
                    rightStart = -rightEncoder.getCurrentPosition();
                    strafeStart = strafeEncoder.getCurrentPosition();
                }
                if(ZONE == 3) {
                    mixer.setMovement(Math.min(0.5, Math.max(((STRAFE_DISTANCE - strafe) / STRAFE_DISTANCE) * 0.5d, 0.2d)), ((STRAFE_DISTANCE - right) / STRAFE_DISTANCE) * 0.0d, 0.0);
                    telemetry.addData("Distance from Target", Math.abs(STRAFE_DISTANCE - strafe));
                    if (Math.abs(Math.abs(STRAFE_DISTANCE) - strafe) < 100) { //if we're within 10 cm of target
                        stage++;
                        //leftStart = leftEncoder.getCurrentPosition();
                        rightStart = -rightEncoder.getCurrentPosition();
                        strafeStart = strafeEncoder.getCurrentPosition();
                    }
                }
            } else if (stage == 2) {
                mixer.setMovement(0, 0, 0);
            }
            telemetry.update();
        }
    }
}

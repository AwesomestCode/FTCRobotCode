package com.example.ftclibexamples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivetrainMixer;
import org.firstinspires.ftc.teamcode.subsystems.OdometryCalculator;

/**
 * This sample shows how to use dead wheels with external encoders
 * paired with motors that don't require encoders.
 * In this sample, we will use the drive motors' encoder
 * ports as they are not needed due to not using the drive encoders.
 * The external encoders we are using are REV through-bore.
 */
@Autonomous
@Disabled
public class PositioningDemo extends LinearOpMode {
    public static final double WHEEL_DIAMETER = 2.0;
    // if needed, one can add a gearing term here
    public static final double TICKS_PER_REV = 8192;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;


    private OdometryCalculator odometry;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx frontLeft = (DcMotorEx) hardwareMap.dcMotor.get("frontLeft");
        DcMotorEx frontRight = (DcMotorEx) hardwareMap.dcMotor.get("frontRight");
        DcMotorEx rearLeft = (DcMotorEx) hardwareMap.dcMotor.get("rearLeft");
        DcMotorEx rearRight = (DcMotorEx) hardwareMap.dcMotor.get("rearRight");


        odometry = new OdometryCalculator(
                frontLeft::getCurrentPosition,
                rearRight::getCurrentPosition,
                frontRight::getCurrentPosition,
                RobotConstants.TRACKWIDTH, RobotConstants.CENTER_WHEEL_OFFSET
        );

        MecanumDrivetrainMixer mixer = new MecanumDrivetrainMixer(frontLeft, frontRight, rearLeft, rearRight);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            // control loop
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            mixer.setMovement(x, y, rx);


            odometry.updatePose(); // update the position
        }
    }
}

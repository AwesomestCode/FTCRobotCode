package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Encoder Read Demo", group="Demo")
@Disabled
public class EncoderReadDemo extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor rawMotor = hardwareMap.get(DcMotor.class, "frontLeft");

        waitForStart();
        //rawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(opModeIsActive()) {
            telemetry.addData("Encoder Position", rawMotor.getCurrentPosition());
            telemetry.update();
            sleep(50);
        }
    }
}

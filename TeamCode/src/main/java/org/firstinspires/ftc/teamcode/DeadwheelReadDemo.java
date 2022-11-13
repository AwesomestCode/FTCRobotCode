package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

@TeleOp(name="Deadwheel Demo")
public class DeadwheelReadDemo extends LinearOpMode {
    public void runOpMode() {
        DcMotorImplEx strafeWheel = hardwareMap.get(DcMotorImplEx.class, "frontLeft");
        int startPos = strafeWheel.getCurrentPosition();
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("Strafe", ((double) (startPos - strafeWheel.getCurrentPosition()) / 8192) * 35 * Math.PI);
            if(gamepad1.a) {
                startPos = strafeWheel.getCurrentPosition();
            }
            telemetry.update();
        }
    }
}

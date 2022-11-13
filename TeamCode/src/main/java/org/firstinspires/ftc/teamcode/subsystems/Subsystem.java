package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public interface Subsystem {
    void run(HardwareMap map, Gamepad gamepad1, Gamepad gamepad2);
}

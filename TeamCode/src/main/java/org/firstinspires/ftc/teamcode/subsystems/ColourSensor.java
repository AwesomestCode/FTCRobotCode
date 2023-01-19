package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

@Config
public
class ColourSensor {

    /** The colorSensor field will contain a reference to our color sensor hardware object */

    private boolean stopRequested = false;
    NormalizedColorSensor colorSensor;
    HardwareMap hardwareMap;

    public ColourSensor(HardwareMap hardwareMap, String name, float gain) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, name);
        this.hardwareMap = hardwareMap;
        colorSensor.setGain(gain);
    }

    /*public void initialise(HardwareMap hardwareMap, Telemetry telemetry) {

        try {
            runSample(hardwareMap, telemetry); // actually execute the sample
        } catch (Exception e) {
            telemetry.addData("Exception", e.toString());
        }
    }*/



    public void requestStop() {
        stopRequested = true;
    }

    public float getGain() {
        return colorSensor.getGain();
    }

    public void setGain(float gain) {
        colorSensor.setGain(gain) ;
    }

    private final float[] rgbValues = new float[3];

    public float[] getRgbValues() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        rgbValues[0] = colors.red;
        rgbValues[1] = colors.green;
        rgbValues[2] = colors.blue;
        return Arrays.copyOf(rgbValues, rgbValues.length);
    }

    private final float[] hsvValues = new float[3];

    public float[] getHsvValues() {
        Color.colorToHSV(colorSensor.getNormalizedColors().toColor(), hsvValues);
        return Arrays.copyOf(hsvValues, hsvValues.length);
    }

    public double getDistance() {
        return ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
    }
}

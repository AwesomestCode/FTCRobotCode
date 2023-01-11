package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;
import java.util.function.DoubleSupplier;

@Config
public class TapePositionSensor {
    public static double kp = 0.5;
    public static double ki = 0;
    public static double kd = 0;
    public enum TapeColour {
        RED,
        BLUE
    }

    private ColourSensor leftSensor;
    private ColourSensor rightSensor;
    private TapeColour colour;
    private PIDController pidController;

    public TapePositionSensor(HardwareMap map, TapeColour colour) {
        leftSensor = new ColourSensor(map, "cs0");
        rightSensor = new ColourSensor(map, "cs1");
        this.colour = colour;
        pidController = new PIDController(this::getPosition);
    }

    public float getPosition() {
        if(colour == TapeColour.RED) {
            return (rightSensor.getHsvValues()[0] - leftSensor.getHsvValues()[0])/ 255.0f;
        } else {
            throw new UnsupportedOperationException("Blue tape not supported yet");
        }
    }

    public PIDController getPidController() {
        return pidController;
    }

    public double getSuggestedPower() {
        return pidController.getOutput();
    }

    public static class PIDController {
        //private double kp;
        //private double ki;
        //private double kd;
        private double lastError;
        private double integral;
        private double derivative;
        private static final double SETPOINT = 0;

        DoubleSupplier input;

        /*public PIDController(double kp, double ki, double kd) {
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
        }*/

        private ElapsedTime timer = new ElapsedTime();

        public PIDController(DoubleSupplier input) {
            timer.reset();
            lastError = SETPOINT - input.getAsDouble();
            this.input = input;
            Timer timer = new Timer();
            timer.schedule(new java.util.TimerTask() {
                @Override
                public void run() {
                    updateValues();
                }
            }, 0, 10);
        }


        public void updateValues() {
            // Calculate error
            double error = SETPOINT - input.getAsDouble();
            // Calculate integral
            integral = this.integral + (timer.nanoseconds() * error)/Math.pow(10, 9);
            // Calculate derivative
            derivative = ((error - lastError) / timer.nanoseconds()) * Math.pow(10, 9);
            // Set last error
            lastError = error;
            timer.reset();
        }

        public void resetPIDState() {
            derivative = 0;
            integral = 0;
            timer.reset();
        }


        public double getOutput() {
            return kp * lastError + ki * integral + kd * derivative;
        }

    }
}

package org.firstinspires.ftc.teamcode.FTClibSubsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.arcrobotics.ftclib.command.SubsystemBase;

public class LinearSlides {
    // PIDF constants - tune these for your robot
    private static final double kP = 0.02;
    private static final double kI = 0.0;
    private static final double kD = 0.02;
    private static final double kF = 0.2;

    private final MotorEx leftSlideMotor; //rightSlideMotor;
    private final PIDFController pidController;
    private final double TICKS_PER_REV = 384.5; // Adjust based on your motor and gearing
    private double MAX_POSITION = 1000;  // Example max position (in encoder ticks)
    private double MIN_POSITION = 0;     // Example min position (in encoder ticks)
    private double targetPosition = 500;

    public LinearSlides(HardwareMap hardwareMap, String leftMotorName, String rightMotorName) {
        // Initialize the motors
        leftSlideMotor = new MotorEx(hardwareMap, leftMotorName);
        //rightSlideMotor = new MotorEx(hardwareMap, rightMotorName);

        // Reverse one motor to ensure both extend the same way
        //rightSlideMotor.setInverted(false);

        // PIDF controller setup
        pidController = new PIDFController(kP, kI, kD, kF);

        // Reset encoders to ensure accurate position tracking
        leftSlideMotor.resetEncoder();
        //rightSlideMotor.resetEncoder();
    }

    public void update(Gamepad gamepad) {
        // Read the current positions of the linear slides
        double currentPosition = (leftSlideMotor.getCurrentPosition()); //+ rightSlideMotor.getCurrentPosition()) / 2;

        // Map buttons to target positions
        if (gamepad.a) {
            targetPosition = MIN_POSITION; // Retract completely
        } else if (gamepad.b) {
            targetPosition = MAX_POSITION / 2; // Extend halfway
        } else if (gamepad.y) {
            targetPosition = MAX_POSITION; // Extend fully
        }

        // Limit target position to the valid range
        targetPosition = Math.max(MIN_POSITION, Math.min(MAX_POSITION, targetPosition));

        // Use PIDF controller to calculate the output power
        double power = pidController.calculate(currentPosition, targetPosition);

        // Apply power to both motors
        leftSlideMotor.set(power);
        //rightSlideMotor.set(power);

        // Optional: Update the display with the current position for debugging
        // telemetry.addData("Current Position", currentPosition);
        // telemetry.addData("Target Position", targetPosition);
    }

    // This function allows setting limits dynamically (if needed)
    public void setLimits(double min, double max) {
        this.MIN_POSITION = min;
        this.MAX_POSITION = max;
    }

    public double getTargetPosition() {
        return targetPosition;
    }


    // Get the current position of the linear slides
    public double getCurrentPosition() {
        return (leftSlideMotor.getCurrentPosition() );//+ rightSlideMotor.getCurrentPosition()) / 2;
    }

    public void updateTarget(int targetPosition) {
        // Clamp the target position within the allowable range
        this.targetPosition = Math.max(MIN_POSITION, Math.min(MAX_POSITION, targetPosition));
    }

}

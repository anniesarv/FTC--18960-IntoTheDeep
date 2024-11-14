/*package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.functions.ProportionalControl;
import com.qualcomm.robotcore.hardware.DcMotor;

public class LinearSlidesSubsystem extends SubsystemBase {

    private final Hardware hardware;
    private final ProportionalControl slidesController;

    // Constants for slide control
    private static final double SLIDES_GAIN = 0.2;
    private static final double SLIDES_ACCEL = 2.0;
    private static final double SLIDES_MAX_AUTO = 0.8;
    private static final double SLIDES_TOLERANCE = 0.5;
    private static final double SLIDES_DEADBAND = 0.2;

    private double targetSlideHeight = 0.0; // Desired slide position (ticks)

    public LinearSlidesSubsystem(Hardware hardware) {
        this.hardware = hardware;

        // Initialize proportional control system for slides
        slidesController = new ProportionalControl(
                SLIDES_GAIN,
                SLIDES_ACCEL,
                SLIDES_MAX_AUTO,
                SLIDES_TOLERANCE,
                SLIDES_DEADBAND,
                false
        );

        // Reset and set slide motors
        resetSlides();
    }

    public void resetSlides() {
        hardware.SlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.SlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.SlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.SlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setSlideHeight(double height, double power) {
        targetSlideHeight = height;
        slidesController.reset(targetSlideHeight, power);
    }

    public void moveSlidesToHeight() {
        double currentLeftHeight = hardware.SlideLeft.getCurrentPosition();
        double currentRightHeight = hardware.SlideRight.getCurrentPosition();
        double averageHeight = (currentLeftHeight + currentRightHeight) / 2;

        // Calculate power using the proportional control system
        double outputPower = slidesController.getOutput(averageHeight);

        // Set power to slide motors
        hardware.SlideLeft.setPower(outputPower);
        hardware.SlideRight.setPower(outputPower);
    }

    public boolean isAtTargetHeight() {
        double currentLeftHeight = hardware.SlideLeft.getCurrentPosition();
        double currentRightHeight = hardware.SlideRight.getCurrentPosition();
        double averageHeight = (currentLeftHeight + currentRightHeight) / 2;

        return slidesController.inPosition(averageHeight);
    }

    public void stopSlides() {
        hardware.SlideLeft.setPower(0);
        hardware.SlideRight.setPower(0);
    }

    @Override
    public void periodic() {
        // Optional: Add telemetry updates or other periodic tasks here
    }
}
*/
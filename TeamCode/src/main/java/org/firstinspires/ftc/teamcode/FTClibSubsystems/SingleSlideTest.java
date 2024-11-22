package org.firstinspires.ftc.teamcode.FTClibSubsystems;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "SingleSlideTest", group = "TeleOp")
public class SingleSlideTest extends OpMode {
    private DcMotor rightLinearSlide;
    private DcMotor leftLinearSlide;

    // Power levels for the linear slide movement
    private static final double SLIDE_UP_POWER = 0.8;
    private static final double SLIDE_DOWN_POWER = -0.8;
    private static final double SLIDE_STOP_POWER = 0.0;

    @Override
    public void init() {
        // Initialize motors
        rightLinearSlide = hardwareMap.get(DcMotor.class, "rightLinearSlide");
        leftLinearSlide = hardwareMap.get(DcMotor.class, "leftLinearSlide");

        // Set initial behavior for linear slides
        rightLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");

    }

    @Override
    public void loop() {
        // Check D-pad input and set power accordingly
        if (gamepad1.dpad_up) {
            // Move slides up
            setLinearSlidePower(SLIDE_UP_POWER);
        } else if (gamepad1.dpad_down) {
            // Move slides down
            setLinearSlidePower(SLIDE_DOWN_POWER);
        } else {
            // Stop slides when no D-pad input
            setLinearSlidePower(SLIDE_STOP_POWER);
        }

        // Display telemetry for debugging
        telemetry.addData("Left Slide Power", leftLinearSlide.getPower());
        telemetry.addData("Right Slide Power", rightLinearSlide.getPower());

        telemetry.update();
    }

    // Helper method to set power to both slides
    private void setLinearSlidePower(double power) {
        rightLinearSlide.setPower(power);
        leftLinearSlide.setPower(-power);
    }
}

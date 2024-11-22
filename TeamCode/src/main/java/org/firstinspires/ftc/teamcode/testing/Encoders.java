package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU.Parameters;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Omni Wheel Encoder Test", group = "Test")
public class Encoders extends LinearOpMode {

    private DcMotor leftLinearSlide;
    private DcMotor rightLinearSlide;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables
        leftLinearSlide = hardwareMap.get(DcMotor.class, "leftLinearSlide");
        rightLinearSlide = hardwareMap.get(DcMotor.class, "rightLinearSlide");

        // Set motors to run using encoders
        leftLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Wait for the IMU to calibrate


        // Check calibration status


        // Wait for the start button to be pressed
        waitForStart();

        // Main loop: Keep running while the OpMode is active
        while (opModeIsActive()) {
            // Read the encoder values
            int leftLinearSlidePos= leftLinearSlide.getCurrentPosition();
            int rightLinearSlidePos = rightLinearSlide.getCurrentPosition();

            // Get orientation angles

            // Send telemetry data to the driver station
            telemetry.addData("left encoder", leftLinearSlidePos);
            telemetry.addData("right encoder", rightLinearSlidePos);
            telemetry.update();

            // Small delay to avoid spamming the telemetry too quickly
            sleep(100);
        }
    }
}

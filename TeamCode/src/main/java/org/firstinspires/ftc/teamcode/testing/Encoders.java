package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU.Parameters;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Omni Wheel Encoder Test", group = "Test")
public class Encoders extends LinearOpMode {

    private DcMotor axial;
    private DcMotor lateral;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables
        axial = hardwareMap.get(DcMotor.class, "axial");
        lateral = hardwareMap.get(DcMotor.class, "lateral");

        // Set motors to run using encoders
        axial.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lateral.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        axial.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lateral.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Wait for the IMU to calibrate


        // Check calibration status


        // Wait for the start button to be pressed
        waitForStart();

        // Main loop: Keep running while the OpMode is active
        while (opModeIsActive()) {
            // Read the encoder values
            int leftLinearSlidePos= axial.getCurrentPosition();
            int rightLinearSlidePos = lateral.getCurrentPosition();

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

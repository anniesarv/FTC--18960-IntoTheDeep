package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Encoder Telemetry", group = "TeleOp")
public class encodertest extends LinearOpMode {

    private DcMotor SlideLeft;
    private DcMotor SlideRight;

    @Override
    public void runOpMode() {
        // Initialize motors
        SlideLeft = hardwareMap.get(DcMotor.class, "Linear Slide Left");
        SlideRight = hardwareMap.get(DcMotor.class, "Linear Slide Right");

        // Reset encoders and set them to RUN_USING_ENCODER mode
        SlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Wait for the game to start
        waitForStart();

        while (opModeIsActive()) {
            // Get encoder values
            int LeftPosition = SlideLeft.getCurrentPosition();
            int RightPosition = SlideRight.getCurrentPosition();

            // Display encoder values on the driver station
            telemetry.addData("Left Encoder", LeftPosition);
            telemetry.addData("Right Encoder", RightPosition);
            telemetry.update();
        }
    }
}

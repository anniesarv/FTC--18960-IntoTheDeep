package org.firstinspires.ftc.teamcode.FTCLibOpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "Motor Test", group = "Test")
public class MotorTest extends LinearOpMode {

    // Declare motor objects
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;

    @Override
    public void runOpMode() {
        // Initialize the hardware
        initHardware(hardwareMap);

        // Wait for the driver to press play
        waitForStart();

        // This is the main loop during teleop
        while (opModeIsActive()) {
            // Example: Set all motors to zero power
            frontLeftMotor.setPower(0); //right back
            frontRightMotor.setPower(0); //right front
            backLeftMotor.setPower(0); //back left
            backRightMotor.setPower(1); //front left

            // Add telemetry to monitor motor states
            telemetry.addData("Motors", "Running");
            telemetry.update();
        }
    }

    // Initialize hardware
    private void initHardware(HardwareMap hwMap) {
        // Map the motors to the configuration in the Control Hub
        frontLeftMotor = hwMap.get(DcMotor.class, "Left Front");
        frontRightMotor = hwMap.get(DcMotor.class, "Right Front");
        backLeftMotor = hwMap.get(DcMotor.class, "Left Back");
        backRightMotor = hwMap.get(DcMotor.class, "Right Back");

        // Set the direction for the motors (adjust based on your robot's wiring)
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        // Set all motors to run without encoders by default
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}

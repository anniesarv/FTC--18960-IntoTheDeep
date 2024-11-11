package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU.Parameters;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Omni Wheel Encoder Test", group = "Test")
public class Encoders extends LinearOpMode {

    // Declare your free-spinning omni wheel DC motors (used for odometry)
    private DcMotor driveEncoder;
    private DcMotor strafeEncoder;
    private BNO055IMU imu;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables
        driveEncoder = hardwareMap.get(DcMotor.class, "axial");
        strafeEncoder = hardwareMap.get(DcMotor.class, "lateral");

        // Set motors to run using encoders
        driveEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafeEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        driveEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        strafeEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // IMU initialization
        Parameters parameters = new Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // Use your calibration file
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Wait for the IMU to calibrate
        while (!imu.isGyroCalibrated() && opModeIsActive()) {
            telemetry.addData("IMU Status", "Calibrating...");
            telemetry.update();
        }

        // Check calibration status
        BNO055IMU.CalibrationData calibrationData = imu.readCalibrationData();
        if (calibrationData.dyMag == 3 && calibrationData.dxGyro == 3 && calibrationData.dzAccel == 3 && calibrationData.dzMag == 3) {
            telemetry.addData("Calibration Status", "Fully calibrated");
        } else {
            telemetry.addData("Calibration Status", "Needs calibration");
        }
        telemetry.update();

        // Wait for the start button to be pressed
        waitForStart();

        // Main loop: Keep running while the OpMode is active
        while (opModeIsActive()) {
            // Read the encoder values
            int DriveOmniPosition = -driveEncoder.getCurrentPosition();
            int StrafeOmniPosition = strafeEncoder.getCurrentPosition();

            // Get orientation angles
            Orientation angles = imu.getAngularOrientation();

            // Send telemetry data to the driver station
            telemetry.addData("Drive Encoder", DriveOmniPosition);
            telemetry.addData("Strafe Wheel Encoder", StrafeOmniPosition);
            telemetry.addData("Heading", angles.firstAngle);
            telemetry.addData("Roll", angles.secondAngle);
            telemetry.addData("Pitch", angles.thirdAngle);
            telemetry.update();

            // Small delay to avoid spamming the telemetry too quickly
            sleep(100);
        }
    }
}

package org.firstinspires.ftc.teamcode.FTClibSubsystems;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Field-Centric xDrive with IMU (Logo Left)")
public class xDrive extends OpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private BNO055IMU imu;
    private double imuHeading;

    @Override
    public void init() {
        // initialize the motors
        frontLeft = hardwareMap.get(DcMotor.class, "Left Front");
        frontRight = hardwareMap.get(DcMotor.class, "Right Front");
        backLeft = hardwareMap.get(DcMotor.class, "Left Back");
        backRight = hardwareMap.get(DcMotor.class, "Right Back");

        // set motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // set all motors to zero power
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // initialize the IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(imuParameters);

        telemetry.addLine("IMU Initialized: Logo Left, Ports Up");
        telemetry.update();
    }

    @Override
    public void loop() {
        // read joystick inputs
        double y = -gamepad1.left_stick_y; // Forward/backward
        double x = gamepad1.left_stick_x;  // Strafe
        double turn = gamepad1.right_stick_x; // Turn

        // get the robot's current heading from the IMU
        imuHeading = imu.getAngularOrientation().firstAngle;

        // convert joystick inputs to field-centric coordinates
        double cosHeading = Math.cos(imuHeading);
        double sinHeading = Math.sin(imuHeading);

        double fieldX = x * cosHeading - y * sinHeading;
        double fieldY = x * sinHeading + y * cosHeading;

        // calculate motor powers for mecanum drive
        double flPower = fieldY + fieldX + turn;
        double frPower = fieldY - fieldX - turn;
        double blPower = fieldY - fieldX + turn;
        double brPower = fieldY + fieldX - turn;

        // normalize powers to prevent clipping
        double maxPower = Math.max(Math.abs(flPower), Math.max(Math.abs(frPower), Math.max(Math.abs(blPower), Math.abs(brPower))));
        if (maxPower > 1.0) {
            flPower /= maxPower;
            frPower /= maxPower;
            blPower /= maxPower;
            brPower /= maxPower;
        }

        // set motor powers
        frontLeft.setPower(flPower);
        frontRight.setPower(frPower);
        backLeft.setPower(blPower);
        backRight.setPower(brPower);

        // display telemetry
        telemetry.addData("IMU Heading (Radians)", imuHeading);
        telemetry.addData("IMU Heading (Degrees)", Math.toDegrees(imuHeading));
        telemetry.addData("FL Power", flPower);
        telemetry.addData("FR Power", frPower);
        telemetry.addData("BL Power", blPower);
        telemetry.addData("BR Power", brPower);
        telemetry.update();
    }
}

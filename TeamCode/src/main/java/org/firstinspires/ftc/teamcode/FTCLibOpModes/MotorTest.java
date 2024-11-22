package org.firstinspires.ftc.teamcode.FTCLibOpModes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MotorTest {

    // Declare motor objects
    public DcMotor frontLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor backLeftMotor = null;
    public DcMotor backRightMotor = null;

    // HardwareMap instance
    private HardwareMap hardwareMap = null;

    // Constructor
    public MotorTest() {}

    // Initialize hardware
    public void init(HardwareMap hwMap) {
        hardwareMap = hwMap;

        // Map the motors to the configuration in the Control Hub
        frontLeftMotor = hardwareMap.get(DcMotor.class, "Front Left");
        frontRightMotor = hardwareMap.get(DcMotor.class, "Front Right");
        backLeftMotor = hardwareMap.get(DcMotor.class, "Left Right");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        // Set the direction for the motors (adjust based on your robot's wiring)
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        frontLeftMotor.setPower(1);
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

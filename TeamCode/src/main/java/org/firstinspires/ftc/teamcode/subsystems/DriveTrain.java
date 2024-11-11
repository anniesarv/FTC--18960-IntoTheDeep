package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class DriveTrain {

    private DcMotor LFM, LBM, RFM, RBM;
    private OpMode opMode;  // will this access telemetry

    // Deadband for small joystick values
    private static final double DEADBAND = 0.1;
    // Power scaling factor
    private double powerScalingFactor = 0.1; // Adjust this value as needed

    public DriveTrain(HardwareMap hardwareMap, OpMode opMode) {
        this.opMode = opMode;  // Store reference to the opMode for telemetry

        // Motor mapping
        LFM = hardwareMap.get(DcMotor.class, "Left Front");
        LBM = hardwareMap.get(DcMotor.class, "Left Back");
        RFM = hardwareMap.get(DcMotor.class, "Right Front");
        RBM = hardwareMap.get(DcMotor.class, "Right Back");

        // Set motor directions
        LFM.setDirection(DcMotorSimple.Direction.REVERSE);
        LBM.setDirection(DcMotorSimple.Direction.REVERSE);
        RFM.setDirection(DcMotorSimple.Direction.REVERSE);
        RBM.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    // Method for Mecanum drive (holonomic control)
    public void drive(double drive, double strafe, double rotate) {

        // Apply deadband to small joystick movements
        drive = applyDeadband(drive);
        strafe = applyDeadband(strafe);
        rotate = applyDeadband(rotate);

        // Calculate motor power
        double LFM_Power = drive + strafe + rotate;
        double LBM_Power = drive - strafe + rotate;
        double RFM_Power = drive - strafe - rotate;
        double RBM_Power = drive + strafe - rotate;

        // Normalize power if any power exceeds 1.0
        double maxPower = Math.max(Math.max(Math.abs(LFM_Power), Math.abs(LBM_Power)),
                Math.max(Math.abs(RFM_Power), Math.abs(RBM_Power)));

        if (maxPower > 1.0) {
            LFM_Power /= maxPower;
            LBM_Power /= maxPower;
            RFM_Power /= maxPower;
            RBM_Power /= maxPower;
        }

        // Apply the power scaling factor
      // LFM_Power *= powerScalingFactor * 6;
        //LBM_Power *= powerScalingFactor * 8 ;
        //RFM_Power *= powerScalingFactor * 9;
       // RBM_Power *= powerScalingFactor * 8  ;

        // Set motor power
        LFM.setPower(LFM_Power);
        LBM.setPower(LBM_Power);
        RFM.setPower(RFM_Power);
        RBM.setPower(RBM_Power);

        // Send motor power values to telemetry for debugging
        opMode.telemetry.addData("LFM Power", LFM_Power);
        opMode.telemetry.addData("LBM Power", LBM_Power);
        opMode.telemetry.addData("RFM Power", RFM_Power);
        opMode.telemetry.addData("RBM Power", RBM_Power);
        opMode.telemetry.update();
    }

    // Apply deadband to avoid tiny unintentional movements
    private double applyDeadband(double value) {
        if (Math.abs(value) < DEADBAND) {
            return 0;
        }
        return value;
    }

    // Stop all motors
    public void stopMotors() {
        LFM.setPower(0);
        LBM.setPower(0);
        RFM.setPower(0);
        RBM.setPower(0);
    }
}

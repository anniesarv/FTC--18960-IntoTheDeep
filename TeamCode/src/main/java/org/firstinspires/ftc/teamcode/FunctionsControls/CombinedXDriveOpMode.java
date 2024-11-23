package org.firstinspires.ftc.teamcode.FunctionsControls;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Combined X-Drive OpMode")
public class CombinedXDriveOpMode extends LinearOpMode {

    private MecanumDrive mecanumDrive;

    @Override
    public void runOpMode() {

        // Initialize motors
        MotorEx frontLeft = new MotorEx(hardwareMap, "Left Front");
        MotorEx frontRight = new MotorEx(hardwareMap, "Right Front");
        MotorEx backLeft = new MotorEx(hardwareMap, "Left Back");
        MotorEx backRight = new MotorEx(hardwareMap, "Right Back");

        // Set motor directions if necessary
        frontLeft.setInverted(false);
        frontRight.setInverted(false);
        backLeft.setInverted(false);
        backRight.setInverted(true);

        // Initialize FTCLib MecanumDrive
        mecanumDrive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        waitForStart();

        // Main control loop
        while (opModeIsActive()) {
            // Get joystick inputs
            double strafe = gamepad1.left_stick_x;
            double forward = -gamepad1.left_stick_y; // Reverse due to joystick axis
            double turn = gamepad1.right_stick_x;

            // Drive the robot
            mecanumDrive.driveRobotCentric(strafe, forward, turn);

            // Telemetry for debugging
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Forward", forward);
            telemetry.addData("Turn", turn);
            telemetry.update();
        }
    }
}

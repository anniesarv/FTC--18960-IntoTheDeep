package org.firstinspires.ftc.teamcode.functions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "X-Drive TeleOp")
public class XDriveTeleOp extends LinearOpMode {


    private PIDController controller;

    private final double ticks_in_inches = 66275 / 52.0; // Full extension ticks per inch

    private DcMotorEx leftLinearSlide;
    private DcMotorEx rightLinearSlide;

    @Override
    public void runOpMode() {

        // Initialize motors
        MotorEx frontLeft = new MotorEx(hardwareMap, "Left Front");
        MotorEx frontRight = new MotorEx(hardwareMap, "Right Front");
        MotorEx backLeft = new MotorEx(hardwareMap, "Left Back");
        MotorEx backRight = new MotorEx(hardwareMap, "Right Back");

        frontLeft.setInverted(false);  // Invert if necessary
        frontRight.setInverted(false);
        backLeft.setInverted(false);
        backRight.setInverted(true);


        // Create the drive subsystem
        XDriveSubsystem xDrive = new XDriveSubsystem(frontLeft, frontRight, backLeft, backRight);

        waitForStart();

        // Main loop
        while (opModeIsActive()) {
            // Get joystick inputs
            double strafe = gamepad1.left_stick_x;
            double forward = -gamepad1.left_stick_y; // Reverse due to joystick axis
            double turn = gamepad1.right_stick_x;

            // Drive the robot
            xDrive.drive(strafe, forward, turn);

            telemetry.addData("Strafe", strafe);
            telemetry.addData("Forward", forward);
            telemetry.addData("Turn", turn);
            telemetry.update();
        }
    }
}

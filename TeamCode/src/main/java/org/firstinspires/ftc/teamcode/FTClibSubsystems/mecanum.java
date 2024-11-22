package org.firstinspires.ftc.teamcode.FTClibSubsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class mecanum extends CommandOpMode {

    private Motor frontLeft, frontRight, backLeft, backRight;
    private MecanumDrive mecanumDrive;

    @Override
    public void initialize() {
        // Initialize motors
        frontLeft = new Motor(hardwareMap, "Left Front", Motor.GoBILDA.RPM_312);
        frontRight = new Motor(hardwareMap, "Right Front", Motor.GoBILDA.RPM_312);
        backLeft = new Motor(hardwareMap, "Left Back", Motor.GoBILDA.RPM_312);
        backRight = new Motor(hardwareMap, "Back Right", Motor.GoBILDA.RPM_312);

        // Set motor directions (reverse as needed)
        frontLeft.setInverted(false);
        backLeft.setInverted(false);

        // Create MecanumDrive object
        //mecanumDrive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        // Telemetry to confirm initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void run() {

        double power = 1;

        backLeft.set(power);

        /*
        // Capture gamepad inputs for driving
        double y = -gamepad1.left_stick_y; // Forward/backward
        double x = gamepad1.left_stick_x; // Strafing
        double turn = gamepad1.right_stick_x; // Turning

        // Drive using MecanumDrive
        mecanumDrive.driveFieldCentric(x, y, turn, 0); // Assuming a heading of 0 for now

        // Update telemetry
        telemetry.addData("X", x);
        telemetry.addData("Y", y);
        telemetry.addData("Turn", turn);
        telemetry.update();

        */

    }
}

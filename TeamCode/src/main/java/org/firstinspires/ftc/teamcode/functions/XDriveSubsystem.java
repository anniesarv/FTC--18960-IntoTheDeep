package org.firstinspires.ftc.teamcode.functions;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.arcrobotics.ftclib.command.SubsystemBase;

public class XDriveSubsystem {


    private MecanumDrive mecanumDrive;

    public XDriveSubsystem(MotorEx frontLeft, MotorEx frontRight, MotorEx backLeft, MotorEx backRight) {
        // Initialize the FTCLib MecanumDrive
        mecanumDrive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
    }

    public void drive(double strafe, double forward, double turn) {
        // Control the robot using joystick values
        mecanumDrive.driveRobotCentric(strafe, forward, turn);
    }
}

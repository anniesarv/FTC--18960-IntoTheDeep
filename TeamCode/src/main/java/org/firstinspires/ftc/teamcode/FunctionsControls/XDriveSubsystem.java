package org.firstinspires.ftc.teamcode.FunctionsControls;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;

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

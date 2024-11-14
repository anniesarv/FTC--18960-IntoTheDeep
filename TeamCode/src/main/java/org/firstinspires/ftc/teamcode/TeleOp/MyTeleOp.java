package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.testing.DriveTrain;

@TeleOp(name = "Holonomic Drive", group = "TeleOp")
public class MyTeleOp extends OpMode {

    private DriveTrain driveTrain;

    @Override
    public void init() {

        driveTrain = new DriveTrain(hardwareMap, this);
    }

    @Override
    public void loop() {
        //gamepad input
        double drive = -gamepad1.left_stick_y;  // forward/back
        double strafe = gamepad1.left_stick_x;  // strafe left/right
        double rotate = gamepad1.right_stick_x; // Rotate left/right

        // Drive the robot
        driveTrain.drive(drive, strafe, rotate);
    }

    @Override
    public void stop() {
        driveTrain.stopMotors();
    }
}

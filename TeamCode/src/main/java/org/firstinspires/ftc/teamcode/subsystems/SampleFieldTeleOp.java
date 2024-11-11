


package org.firstinspires.ftc.teamcode.subsystems;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.SimplifiedOdometryRobot;


/*
 * This OpMode illustrates a teleop OpMode for an Omni robot.
 * An external "Robot" class is used to manage all motor/sensor interfaces, and to assist driving functions.
 * The IMU gyro is used to stabilize the heading when the operator is not requesting a turn.
 */

@TeleOp(name="Sample TeleOp", group = "Mr. Phil")
public class SampleFieldTeleOp extends LinearOpMode
{
    final double SAFE_DRIVE_SPEED   = 0.8 ; // Adjust this to your robot and your driver.  Slower usually means more accuracy.  Max value = 1.0
    final double SAFE_STRAFE_SPEED  = 0.8 ; // Adjust this to your robot and your driver.  Slower usually means more accuracy.  Max value = 1.0
    final double SAFE_YAW_SPEED     = 0.5 ; // Adjust this to your robot and your driver.  Slower usually means more accuracy.  Max value = 1.0
    final double HEADING_HOLD_TIME  = 10.0 ; // How long to hold heading once all driver input stops. (This Avoids effects of Gyro Drift)

    // local parameters
    ElapsedTime stopTime   = new ElapsedTime();  // Use for timeouts.
    boolean autoHeading    = false; // used to indicate when heading should be locked.

    // get robot instance
    SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);

    @Override
    public void runOpMode() {
        // initialize the drive hardware & turn on telemetry
        robot.initialize(true);

        // Wait for driver to press start
        while(opModeInInit()) {
            telemetry.addData(">", "Touch Play to drive");

            // read and display sensor data
            robot.readSensors();
            telemetry.update();
        }

        while (opModeIsActive()) {
            robot.readSensors();

            // allows the driver to reset the gyro by pressing both small gamepad buttons
            if(gamepad1.options && gamepad1.share){
                robot.resetHeading();
                robot.resetOdometry();
            }

            // read joystick values and scale according to limits set at top of this file
            double drive  = -gamepad1.left_stick_y * SAFE_DRIVE_SPEED;      // Fwd/back on left stick
            double strafe = -gamepad1.left_stick_x * SAFE_STRAFE_SPEED;     // Left/Right on left stick
            double yaw    = -gamepad1.right_stick_x * SAFE_YAW_SPEED;       // Rotate on right stick

            // get the robot's current heading (in radians) for field-centric calculations
            double robotHeading = Math.toRadians(robot.getHeading());

            // adjust joystick inputs for field-centric control using rotation matrix
            double tempDrive = drive * Math.cos(robotHeading) + strafe * Math.sin(robotHeading);
            double tempStrafe = -drive * Math.sin(robotHeading) + strafe * Math.cos(robotHeading);

            drive = tempDrive;
            strafe = tempStrafe;

            // OR... For special conditions, Use the DPAD to make pure orthogonal motions
            if (gamepad1.dpad_left) {
                strafe = SAFE_DRIVE_SPEED / 2.0;
            } else if (gamepad1.dpad_right) {
                strafe = -SAFE_DRIVE_SPEED / 2.0;
            } else if (gamepad1.dpad_up) {
                drive = SAFE_DRIVE_SPEED / 2.0;
            } else if (gamepad1.dpad_down) {
                drive = -SAFE_STRAFE_SPEED / 2.0;
            }

            // Heading lock control
            if (Math.abs(yaw) > 0.05) {
                autoHeading = false;
            } else {
                if (!autoHeading && Math.abs(robot.getTurnRate()) < 2.0) {
                    robot.yawController.reset(robot.getHeading());
                    autoHeading = true;
                }
            }

            if (autoHeading) {
                yaw = robot.yawController.getOutput(robot.getHeading());
            }

            telemetry.addData("Drive", drive);
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Yaw", yaw);
            telemetry.update();

            // drive the wheels based on the adjusted drive, strafe, and yaw values
            robot.moveRobot(drive, strafe, yaw);

            if ((drive == 0) && (strafe == 0) && (yaw == 0)) {
                if (stopTime.time() > HEADING_HOLD_TIME) {
                    robot.yawController.reset(robot.getHeading());
                }
            } else {
                stopTime.reset();
            }
        }
    }

}

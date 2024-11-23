package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



@Autonomous
       // (name="Sample Autonomous", group = "Mr. Phil")
public class SampleAuto extends LinearOpMode
{
    // get an instance of the "Robot" class.
    private SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);
    private AutoSlideSubsystem autoSlideSubsystem;


    @Override public void runOpMode()
    {
        // Initialize the robot hardware & Turn on telemetry
        robot.initialize(true);
        autoSlideSubsystem = new AutoSlideSubsystem(hardwareMap, new MultipleTelemetry(telemetry));


        // Wait for driver to press start
        telemetry.addData(">", "Touch Play to run Auto");
        telemetry.update();

        waitForStart();
        robot.resetHeading();  // Reset heading to set a baseline for Auto

        // Run Auto if stop was not pressed.
        if (opModeIsActive())
        {

            robot.strafe(  24, 0.60, 1);
            sleep(500);
            autoSlideSubsystem.setTarget(15000);

            //robot.turnTo(180, 0.45, 1);
            robot.drive(24, 0.6, 1);

            while (opModeIsActive()) {
                autoSlideSubsystem.update();
                telemetry.addLine("Running...");
                telemetry.update();
            }





            sleep(500);

            robot.stopRobot();

        }
    }
}
package org.firstinspires.ftc.teamcode.FTCLibOpModes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; // Use LinearOpMode directly
import org.firstinspires.ftc.teamcode.FTClibSubsystems.LinearSlides;



@Autonomous(name = "LinearSlides Test", group = "Test")
public class  LinearSlidesTele extends LinearOpMode {
    private LinearSlides linearSlides;

    @Override
    public void runOpMode() {
        linearSlides = new LinearSlides(hardwareMap, "leftLinearSlide", "rightLinearSlide");

        waitForStart();

        linearSlides.setLimits(0, 5000); // Example limits
        linearSlides.updateTarget(2500); // Move halfway
        sleep(3000); // Wait for 3 seconds
        linearSlides.updateTarget(0); // Retract completely
        sleep(3000);
    }
}



/*
@TeleOp
public class LinearSlidesTeleOp extends LinearOpMode {

    // Declare the LinearSlides subsystem
    private LinearSlides linearSlides;

@Override
    public void runOpMode() {
        // Initialize hardware and LinearSlides subsystem
        linearSlides = new LinearSlides(hardwareMap, "leftLinearSlide", "rightLinearSlide");

        // Wait for the start button to be pressed
        waitForStart();

        // Run the loop until the opmode is stopped
        while (opModeIsActive()) {
            // Update the linear slides based on the gamepad inputs
            linearSlides.update(gamepad1);

            // Optionally: Display telemetry for debugging
            telemetry.addData("Current Position", linearSlides.getCurrentPosition());
            telemetry.addData("Target Position", linearSlides.getTargetPosition());
            telemetry.update();


            // Give control back to the robot's other systems
            sleep(20);  // Small delay to avoid overloading the loop
        }
    }
}

 */
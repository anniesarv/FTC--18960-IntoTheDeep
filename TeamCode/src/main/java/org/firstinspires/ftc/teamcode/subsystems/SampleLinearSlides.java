package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;


public class SampleLinearSlides {

    private final double ENCODER_INCHES_PER_COUNT =  2691.77854671;//3076.3183391; //endcoder tics per in traveller DO THIS MATH ANNIE
    private final boolean INVERT_LEFT_SLIDE_ENCODER = false; //left slides encoder tics might be backwards
    private final boolean INVERT_RIGHT_SLIDE_ENCODER = false; //right slide might be backwards
    private DcMotor linearSlideRight;
    private DcMotor linearSlideLeft;
    private LinearOpMode myOpMode;
    private ElapsedTime holdTimer = new ElapsedTime(); //anything that needs timer i hope i did this right

    private int rawLeftSlideHeight = 0;
    private int rawRightSlideHeight = 0;
    private boolean showTelemetry = false;

    private static final double SLIDES_GAIN = 0.2;
    private static final double SLIDES_ACCEL = 2; //acceleration limit for slides 1 and 2 together
    private static final double SLIDES_DEADBAND = 0.2; //deadband for slides controller, error less than this means zero output
    private static final double SLIDES_TOLERANCE = 0.5; //slides are "inPosition"  if error is < +- this amount
    private static final double SLIDES_MAX_AUTO = 0.8;// max speed for linear slides in auto
    //private static final double SLIDES_MAX_TELE = 0.8; //max speed for linear slides in teleop
    //private static final double SLIDES_MAX_HEIGHT = 100; //

    public double slideDistance = 0; //distance gone upwards sub as drive distance
    public double encoderTics = 3076.3183391;//sub as heading


    public ProportionalControl slidesController = new ProportionalControl(SLIDES_GAIN, SLIDES_ACCEL, SLIDES_MAX_AUTO, SLIDES_TOLERANCE, SLIDES_DEADBAND, false);

    public SampleLinearSlides(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    private void setupBulkCaching() {
        List<LynxModule> allHubs = myOpMode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public void initialize (boolean showTelemetry) {
        linearSlideRight = setUpSlideMotor("Linear Slide Right", DcMotor.Direction.FORWARD);
        linearSlideLeft = setUpSlideMotor("Linear Slide Left", DcMotor.Direction.REVERSE);

        setupBulkCaching(); // call the helper method for bulk caching

        this.showTelemetry = showTelemetry;

        double TELE_SUPPORT = 0.009;

    }



    //sub setupdrivemotor
    private DcMotor setUpSlideMotor(String deviceName, DcMotor.Direction direction) {
        DcMotor lMotor = myOpMode.hardwareMap.get(DcMotor.class, deviceName);
        lMotor.setDirection(direction);
        lMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return lMotor;
    }


    public boolean getSlideDistance() {
        rawLeftSlideHeight = (linearSlideLeft.getCurrentPosition() * (INVERT_LEFT_SLIDE_ENCODER ? -1 : 1));
        rawRightSlideHeight = (linearSlideRight.getCurrentPosition() * (INVERT_RIGHT_SLIDE_ENCODER ? -1 : 1));
        slideDistance = ((rawLeftSlideHeight + rawRightSlideHeight)) / 2;

        if (showTelemetry) {
            myOpMode.telemetry.addData("Left Slide Height", rawLeftSlideHeight);
            myOpMode.telemetry.addData("Right Slide Height", rawRightSlideHeight);
            myOpMode.telemetry.update();
        }

        return true; //this is so function can be included in condition
    }


    //subasdrive
    public void driveSlidesUp (double slidesHeight, double power, double holdTime) {

        slidesController.reset(slidesHeight, power);  // Set desired slide height based on controller input
        holdTimer.reset();

        // calculate encoder ticks needed
        double targetTicks = 3076.3183391;

        while (myOpMode.opModeIsActive()) {
            // get current slide distance in ticks
            getSlideDistance();

            myOpMode.telemetry.addData("Current Slide Distance (ticks)", slideDistance);
            myOpMode.telemetry.addData("Target Slide Distance (ticks)", targetTicks);
            myOpMode.telemetry.addData("Error (ticks)", targetTicks - slideDistance);
            myOpMode.telemetry.update();

            // Use the proportional control system to adjust power
            double outputPower = slidesController.getOutput(slideDistance);

            // Move slides based on output power
            moveSlides(outputPower);

            // Check if slides have reached the target position
            if (Math.abs(slideDistance - targetTicks) <= slidesController.tolerance || slidesController.inPosition()) {
                if (holdTimer.time() > holdTime) {
                    break; // Exit loop when target is reached and hold time is met
                }
            } else {
                // Reset the hold timer if we're not in position
                holdTimer.reset();
            }

            myOpMode.sleep(10);  // Delay for loop iteration
        }

        stopSlides();  // Stop the motors once target is reached
    }

    public void moveSlides(double up) {

    linearSlideLeft.setPower(up);
    linearSlideRight.setPower(up);

  /*  if (showTelemetry) {
        myOpMode.telemetry.addData("hey look your slides might be running this is cute", linearSlide1, linearSlide2);
    } */


}

public void stopSlides() { moveSlides(0); }




}
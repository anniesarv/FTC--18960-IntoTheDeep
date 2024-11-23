
package org.firstinspires.ftc.teamcode.FunctionsControls;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;


@TeleOp
public class TeleControllerWithSlides extends OpMode {
    //private SlideSubsystem linearSlideSubsystem;
    private ServoSubsystem servoSubsystem;

    private DcMotorEx actuator;
    private CRServo  oCRServoLeft, oCRServoRight;
    private PIDController controller;
    public static double p = 0.001, i = 0, d =0.00001 ;
    public static double kg = 0.1; // gravity compensation
    public static int target = 1000; // target position in encoder ticks

    private final double ticks_in_inches = 66275 / 52.0; // Full extension ticks per inch

    private DcMotorEx leftLinearSlide;
    private DcMotorEx rightLinearSlide;




    @Override

    public void init() {
        // Initialize the SlideSubsystem with necessary parameters

        //linearSlideSubsystem = new SlideSubsystem(hardwareMap, 0.001, 0, 0.00001);


        // Initialize the ServoSubsystem
        servoSubsystem = new ServoSubsystem(hardwareMap);

        // Initialize the actuator and servos
        actuator = hardwareMap.get(DcMotorEx.class, "axial");
        oCRServoLeft = hardwareMap.get(CRServo.class, "oCRServoLeft");
        oCRServoRight = hardwareMap.get(CRServo.class, "oCRServoRight");

        // Additional setup for motors/servos if needed
        actuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //linearSlideSubsystem.setTarget(0);
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftLinearSlide = hardwareMap.get(DcMotorEx.class, "leftLinearSlide");
        rightLinearSlide = hardwareMap.get(DcMotorEx.class, "rightLinearSlide");



        leftLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftLinearSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightLinearSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftLinearSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightLinearSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        leftLinearSlide.setDirection(DcMotorEx.Direction.REVERSE);
        rightLinearSlide.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    public void loop() {
        // Update PID values

        controller.setPID(p, i, d); // Update PID values from dashboard


        // Get dpad input from gamepad
        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;

        int leftSlidePos = leftLinearSlide.getCurrentPosition(); // Negate for correct direction
        int rightSlidePos = -rightLinearSlide.getCurrentPosition();

        // Average slide positions
        int slidesPos = (leftSlidePos );//+ rightSlidePos) / 2;


        //linearSlideSubsystem.controlWithDpad(dpadUp, dpadDown);

        // Linear slide controls
        //linearSlideSubsystem.controlSlides();


        //target on amanda controller for intake
        if (gamepad1.right_bumper) {
            target = 3150;
        }


        // Actuator controls
        if (gamepad2.dpad_left) {
            actuator.setPower(1.0); // Extend
        } else if (gamepad2.dpad_right) {
            actuator.setPower(-1.0); // Retract
        } else {
            actuator.setPower(0); // Stop
        }

        if (gamepad1.dpad_up) {
            target += 250;
        }
        else if (gamepad1.dpad_down) {
            target -= 250;
        }

        target = Math.max(2500, Math.min(target, 65000));
        double pid = controller.calculate(slidesPos, target);
        double power = pid + kg;

        leftLinearSlide.setPower(power);
        rightLinearSlide.setPower(power);





        // Servo controls
        servoSubsystem.setIntakeWristPosition(gamepad2.right_bumper, gamepad2.left_bumper);
        servoSubsystem.controlIntake(gamepad2.x, gamepad2.y);

        // Refactored logic for outtake CRServos
        if (gamepad2.a || gamepad2.b) {
            servoSubsystem.controlOuttake(gamepad2.a, gamepad2.b); // Prioritize A/B
        } else if (gamepad2.dpad_down) {
            servoSubsystem.servoTransfer(true); // Fall back to D-pad Down
        } else {
            // Stop servos if no input
            servoSubsystem.controlOuttake(false, false);
            servoSubsystem.servoTransfer(false);
        }

        if (gamepad2.dpad_up) {
            servoSubsystem.wristTransfer(true);
        }

        // Telemetry for debugging
       // telemetry.addData("Slide Target", linearSlideSubsystem.getTarget());
        telemetry.addData("oCRServoLeft Power", oCRServoLeft.getPower());
        telemetry.addData("oCRServoRight Power", oCRServoRight.getPower());
        telemetry.update();
        telemetry.addData("Left Slide Position", leftSlidePos);
        telemetry.addData("Right Slide Position", rightSlidePos);
        telemetry.addData("Target Position", target);
        telemetry.addData("Slide Position (Avg)", slidesPos);
        telemetry.addData("PID Output", pid);
        telemetry.addData("Motor Power", power);
        telemetry.update();
    }


}



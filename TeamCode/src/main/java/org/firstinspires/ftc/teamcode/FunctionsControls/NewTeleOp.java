
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



import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;



@TeleOp
public class NewTeleOp extends OpMode {
    //private SlideSubsystem linearSlideSubsystem;
    private DcMotorEx actuator;
    private CRServo  oCRServoLeft, oCRServoRight;
    private DcMotorEx leftLinearSlide;
    private DcMotorEx rightLinearSlide;
    private Servo iServoLeft, iServoRight;
    private CRServo iCRServoLeft, iCRServoRight;

    private ElapsedTime timer = new ElapsedTime();
    private boolean isSequenceActive = false;
    private int sequenceStep = 0;

    private PIDController controller;
    public static double p = 0.001, i = 0, d =0.00001 ;
    public static double kg = 0.1; // gravity compensation
    public static int target = 1000; // target position in encoder ticks


    @Override

    public void init() {
        // Initialize the SlideSubsystem with necessary parameters

        //linearSlideSubsystem = new SlideSubsystem(hardwareMap, 0.001, 0, 0.00001);


        // Initialize the ServoSubsystem

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

        iServoLeft = hardwareMap.get(Servo.class, "iServoLeft");
        iServoRight = hardwareMap.get(Servo.class, "iServoRight");
        iCRServoLeft = hardwareMap.get(CRServo.class, "iCRServoLeft");
        iCRServoRight = hardwareMap.get(CRServo.class, "iCRServoRight");
        oCRServoLeft = hardwareMap.get(CRServo.class, "oCRServoLeft");
        oCRServoRight = hardwareMap.get(CRServo.class, "oCRServoRight");



    }

    public void loop() {

        //LINEAR SLIDES
        controller.setPID(p, i, d); // Update PID values from dashboard
        int leftSlidePos = leftLinearSlide.getCurrentPosition(); // Negate for correct direction
        int rightSlidePos = -rightLinearSlide.getCurrentPosition();
        int slidesPos = (leftSlidePos );//+ rightSlidePos) / 2;

        //FOR DRIVE CONTROLLER
        //target on amanda controller for intake
        if (gamepad1.right_bumper) {
            target = 3150;
        }

        //linear slide movement
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



        //FOR FUNCTIONS CONTROLLER

        //linear actuator
        if (gamepad2.dpad_left) {
            actuator.setPower(1.0); // Extend
        } else if (gamepad2.dpad_right) {
            actuator.setPower(-1.0); // Retract
        } else {
            actuator.setPower(0); // Stop
        }

        //intake servos


        if (gamepad2.x) {
            iCRServoLeft.setPower(1.0);
            iCRServoRight.setPower(-1.0);
        } else if (gamepad2.y) {
            iCRServoLeft.setPower(-1.0);
            iCRServoRight.setPower(1.0);
        } else {
            iCRServoLeft.setPower(0);
            iCRServoRight.setPower(0);
        }

        //outtake servos
        if (gamepad2.a) {
            oCRServoLeft.setPower(1);
            oCRServoRight.setPower(-1);
        } else if (gamepad2.b) {
            oCRServoLeft.setPower(-1);
            oCRServoRight.setPower(1);
        } else {
            oCRServoLeft.setPower(0);
            oCRServoRight.setPower(0);
        }

        //intake servo WRIST
        if (gamepad2.right_bumper) {
            iServoLeft.setPosition(0.12);
            iServoRight.setPosition(0.8);
        } else if (gamepad2.left_bumper) {
            iServoLeft.setPosition(1);
            iServoRight.setPosition(0);
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



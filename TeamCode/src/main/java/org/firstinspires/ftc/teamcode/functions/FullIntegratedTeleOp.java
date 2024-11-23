/*package org.firstinspires.ftc.teamcode.FunctionsControls;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Integrated TeleOp")
public class FullIntegratedTeleOp extends LinearOpMode {

    private SlideSubsystem linearSlideSubsystem;
    private ServoSubsystem servoSubsystem;
    private MotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx actuator, leftLinearSlide, rightLinearSlide;
    private CRServo oCRServoLeft, oCRServoRight;

    private PIDController slideController;
    public static double p = 0.001, i = 0, d = 0.00001, kg = 0.1;
    private int target = 1000; // Initial target position for the slides

    @Override
    public void runOpMode() {
        // Initialize hardware
        servoSubsystem = new ServoSubsystem(hardwareMap);

        oCRServoLeft = hardwareMap.get(CRServo.class, "oCRServoLeft");
        oCRServoRight = hardwareMap.get(CRServo.class, "oCRServoRight");
        actuator = hardwareMap.get(DcMotorEx.class, "axial");

        actuator.setDirection(DcMotorSimple.Direction.FORWARD);
        actuator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize drive motors
        frontLeft = new MotorEx(hardwareMap, "Left Front");
        frontRight = new MotorEx(hardwareMap, "Right Front");
        backLeft = new MotorEx(hardwareMap, "Left Back");
        backRight = new MotorEx(hardwareMap, "Right Back");

        frontLeft.setInverted(false);
        frontRight.setInverted(false);
        backLeft.setInverted(false);
        backRight.setInverted(true);

        XDriveSubsystem xDrive = new XDriveSubsystem(frontLeft, frontRight, backLeft, backRight);

        // Initialize linear slides
        leftLinearSlide = hardwareMap.get(DcMotorEx.class, "leftLinearSlide");
        rightLinearSlide = hardwareMap.get(DcMotorEx.class, "rightLinearSlide");

        leftLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLinearSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightLinearSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftLinearSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightLinearSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        leftLinearSlide.setDirection(DcMotorEx.Direction.REVERSE);
        rightLinearSlide.setDirection(DcMotorEx.Direction.FORWARD);

        slideController = new PIDController(p, i, d);

        waitForStart();

        // Main loop
        while (opModeIsActive()) {
            // **Drivetrain Controls**
            double strafe = gamepad1.left_stick_x;
            double forward = -gamepad1.left_stick_y; // Reverse due to joystick axis
            double turn = gamepad1.right_stick_x;

            xDrive.drive(strafe, forward, turn);

            // **Linear Slide Controls**
            slideController.setPID(p, i, d);

            int leftSlidePos = leftLinearSlide.getCurrentPosition();
            int rightSlidePos = rightLinearSlide.getCurrentPosition();
            int slidesPos = (leftSlidePos + rightSlidePos) / 2;

            if (gamepad1.dpad_up) {
                target += 250; // Increase target
            } else if (gamepad1.dpad_down) {
                target -= 250; // Decrease target
            }

            // Limit the target position
            target = Math.max(2500, Math.min(target, 65000));

            // PID output
            double pidOutput = slideController.calculate(slidesPos, target);

            // Gravity compensation and apply power
            double slidePower = pidOutput + kg;
            leftLinearSlide.setPower(slidePower);
            rightLinearSlide.setPower(slidePower);

            // **Telemetry for Debugging**
            telemetry.addData("Slide Target", target);
            telemetry.addData("Slide Position", slidesPos);
            telemetry.addData("PID Output", pidOutput);
            telemetry.addData("Slide Power", slidePower);
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Forward", forward);
            telemetry.addData("Turn", turn);
            telemetry.update();
        }
    }
}


 */
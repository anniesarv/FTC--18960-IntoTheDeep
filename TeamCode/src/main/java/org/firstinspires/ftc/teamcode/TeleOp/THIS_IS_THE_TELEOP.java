package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "THIS_IS_THE_TELEOP")
public class THIS_IS_THE_TELEOP extends OpMode {

    // Subsystems and hardware components
    private MecanumDrive mecanumDrive; // Drivebase
    private DcMotorEx actuator;
    private CRServo oCRServoLeft, oCRServoRight;
    private DcMotorEx leftLinearSlide;
    private DcMotorEx rightLinearSlide;
    private Servo iServoLeft, iServoRight;
    private CRServo iCRServoLeft, iCRServoRight;

    private PIDController controller;
    private ElapsedTime timer = new ElapsedTime();

    public static double p = 0.001, i = 0, d = 0.00001;
    public static double kg = 0.1; // Gravity compensation
    public static int target = 1000; // Linear slide target position in encoder ticks

    @Override
    public void init() {
        // Initialize drivebase motors
        MotorEx frontLeft = new MotorEx(hardwareMap, "Left Front");
        MotorEx frontRight = new MotorEx(hardwareMap, "Right Front");
        MotorEx backLeft = new MotorEx(hardwareMap, "Left Back");
        MotorEx backRight = new MotorEx(hardwareMap, "Right Back");

        // Set motor directions for drivebase
        frontLeft.setInverted(false);
        frontRight.setInverted(false);
        backLeft.setInverted(false);
        backRight.setInverted(true);

        // Initialize FTCLib MecanumDrive
        mecanumDrive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

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
        rightLinearSlide.setDirection(DcMotorSimple.Direction.FORWARD);

        // Initialize other components
        actuator = hardwareMap.get(DcMotorEx.class, "axial");
        actuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        oCRServoLeft = hardwareMap.get(CRServo.class, "oCRServoLeft");
        oCRServoRight = hardwareMap.get(CRServo.class, "oCRServoRight");

        iServoLeft = hardwareMap.get(Servo.class, "iServoLeft");
        iServoRight = hardwareMap.get(Servo.class, "iServoRight");
        iCRServoLeft = hardwareMap.get(CRServo.class, "iCRServoLeft");
        iCRServoRight = hardwareMap.get(CRServo.class, "iCRServoRight");

        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        // Drivebase control (gamepad1)
        double strafe = gamepad1.left_stick_x;
        double forward = -gamepad1.left_stick_y; // Reverse due to joystick axis
        double turn = gamepad1.right_stick_x;
        mecanumDrive.driveRobotCentric(strafe, forward, turn);

        turn = Math.max(-0.5, Math.min(0.5, turn));


        // Linear slides control (gamepad1)
        controller.setPID(p, i, d);
        int leftSlidePos = leftLinearSlide.getCurrentPosition();
        int rightSlidePos = -rightLinearSlide.getCurrentPosition();
        int slidesPos = leftSlidePos; // Average position if needed

        if (gamepad1.right_bumper) target = 3150;
        if (gamepad1.dpad_up) target += 250;
        else if (gamepad1.dpad_down) target -= 250;
        target = Math.max(-10000, Math.min(target, 67000));

        if (gamepad1.x) {
            leftLinearSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rightLinearSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            leftLinearSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            rightLinearSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            target = 0; // Reset target to match the new encoder state
        }
        if (gamepad1.b) {
            target = 2675;
        }


        double pid = controller.calculate(slidesPos, target);
        double power = pid + kg;
        leftLinearSlide.setPower(power);
        rightLinearSlide.setPower(power);

        // Actuator control (gamepad2)
        if (gamepad2.dpad_left) actuator.setPower(1.0); // Extend
        else if (gamepad2.dpad_right) actuator.setPower(-1.0); // Retract
        else actuator.setPower(0); // Stop

        // Intake servo control (gamepad2)
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

        // Outtake servo control (gamepad2)
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

        // Intake servo wrist control (gamepad2)
        if (gamepad2.right_bumper) {
            iServoLeft.setPosition(0.16);
            iServoRight.setPosition(0.84);
        } else if (gamepad2.left_bumper) {
            iServoLeft.setPosition(1);
            iServoRight.setPosition(0);
        }

        // Telemetry
        telemetry.addData("Strafe", strafe);
        telemetry.addData("Forward", forward);
        telemetry.addData("Turn", turn);
        telemetry.addData("Left Slide Position", leftSlidePos);
        telemetry.addData("Right Slide Position", rightSlidePos);
        telemetry.addData("Target Position", target);
        telemetry.addData("Slide Position (Avg)", slidesPos);
        telemetry.addData("PID Output", pid);
        telemetry.addData("Motor Power", power);
        telemetry.update();
    }
}

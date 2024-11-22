package org.firstinspires.ftc.teamcode.FunctionsControls;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Integrated TeleOp")
public class IntegratedTeleOp extends LinearOpMode {

    private SlideSubsystem linearSlideSubsystem;
    private ServoSubsystem servoSubsystem;
    private MotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx actuator;
    private CRServo oCRServoLeft, oCRServoRight;

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

        frontLeft.setInverted(false);  // Invert if necessary
        frontRight.setInverted(false);
        backLeft.setInverted(false);
        backRight.setInverted(true);

        XDriveSubsystem xDrive = new XDriveSubsystem(frontLeft, frontRight, backLeft, backRight);

        waitForStart();

        // Main loop
        while (opModeIsActive()) {
            // **Drivetrain Controls**
            double strafe = gamepad1.left_stick_x;
            double forward = -gamepad1.left_stick_y; // Reverse due to joystick axis
            double turn = gamepad1.right_stick_x;

            xDrive.drive(strafe, forward, turn);

            // **Linear Slide Controls**
            linearSlideSubsystem.updatePID(TELEPIDCOLLAB.p, TELEPIDCOLLAB.i, TELEPIDCOLLAB.d);
            linearSlideSubsystem.controlSlides();

            if (gamepad2.dpad_up) {
                linearSlideSubsystem.setTarget(3150);
            }

            // **Actuator Controls**
            if (gamepad2.dpad_left) {
                actuator.setPower(1.0); // Extend
            } else if (gamepad2.dpad_right) {
                actuator.setPower(-1.0); // Retract
            } else {
                actuator.setPower(0); // Stop
            }

            // **Servo Controls**
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

            // **Telemetry for Debugging**
            telemetry.addData("Slide Target", linearSlideSubsystem.getTarget());
            telemetry.addData("oCRServoLeft Power", oCRServoLeft.getPower());
            telemetry.addData("oCRServoRight Power", oCRServoRight.getPower());
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Forward", forward);
            telemetry.addData("Turn", turn);
            telemetry.update();
        }
    }
}

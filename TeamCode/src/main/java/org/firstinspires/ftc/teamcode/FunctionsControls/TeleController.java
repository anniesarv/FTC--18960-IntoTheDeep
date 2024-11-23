
package org.firstinspires.ftc.teamcode.FunctionsControls;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.functions.TELEPIDCOLLAB;


@TeleOp
public class TeleController extends OpMode {
    private SlideSubsystem linearSlideSubsystem;
    private ServoSubsystem servoSubsystem;

    private DcMotorEx actuator;
    private CRServo  oCRServoLeft, oCRServoRight;



    @Override

    public void init() {
        // Initialize the SlideSubsystem with necessary parameters
        linearSlideSubsystem = new SlideSubsystem(hardwareMap, TELEPIDCOLLAB.p, TELEPIDCOLLAB.i, TELEPIDCOLLAB.d);

        // Initialize the ServoSubsystem
        servoSubsystem = new ServoSubsystem(hardwareMap);

        // Initialize the actuator and servos
        actuator = hardwareMap.get(DcMotorEx.class, "axial");
        oCRServoLeft = hardwareMap.get(CRServo.class, "oCRServoLeft");
        oCRServoRight = hardwareMap.get(CRServo.class, "oCRServoRight");

        // Additional setup for motors/servos if needed
        actuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void loop() {
        // Update PID values
        linearSlideSubsystem.updatePID(TELEPIDCOLLAB.p, TELEPIDCOLLAB.i, TELEPIDCOLLAB.d);

        // Linear slide controls
        linearSlideSubsystem.controlSlides();

        if (gamepad1.right_bumper) {
            linearSlideSubsystem.setTarget(3150);
        }


        // Actuator controls
        if (gamepad2.dpad_left) {
            actuator.setPower(1.0); // Extend
        } else if (gamepad2.dpad_right) {
            actuator.setPower(-1.0); // Retract
        } else {
            actuator.setPower(0); // Stop
        }

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
        telemetry.addData("Slide Target", linearSlideSubsystem.getTarget());
        telemetry.addData("oCRServoLeft Power", oCRServoLeft.getPower());
        telemetry.addData("oCRServoRight Power", oCRServoRight.getPower());
        telemetry.update();
    }


}



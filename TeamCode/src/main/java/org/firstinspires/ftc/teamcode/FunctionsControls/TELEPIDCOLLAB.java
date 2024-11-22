
package org.firstinspires.ftc.teamcode.FunctionsControls;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@Config
@TeleOp
public class TELEPIDCOLLAB extends OpMode {
    private PIDController controller;
    private DcMotorEx Actuator;

    public static double p = 0.001, i = 0, d =0.00001 ;
    public static double kg = 0.1; // gravity compensation
    public static int target = 1000; // target position in encoder ticks

    private final double ticks_in_inches = 66275 / 52.0; // Full extension ticks per inch

    private DcMotorEx leftLinearSlide;
    private DcMotorEx rightLinearSlide;
    private Servo iServoLeft;
    private Servo iServoRight;
    private CRServo iCRServoLeft;
    private CRServo iCRServoRight;
    private CRServo oCRServoRight;
    private CRServo oCRServoLeft;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftLinearSlide = hardwareMap.get(DcMotorEx.class, "leftLinearSlide");
        rightLinearSlide = hardwareMap.get(DcMotorEx.class, "rightLinearSlide");
        Actuator = hardwareMap.get(DcMotorEx.class, "axial");

        Actuator.setDirection(DcMotorSimple.Direction.FORWARD);
        Actuator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


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

    @Override
    public void loop() {
        controller.setPID(p, i, d); // Update PID values from dashboard

        //slides
        int leftSlidePos = leftLinearSlide.getCurrentPosition(); // Negate for correct direction
        int rightSlidePos = -rightLinearSlide.getCurrentPosition();

        // Average slide positions
        int slidesPos = (leftSlidePos );//+ rightSlidePos) / 2;

        //dpad input i hope it works
        if (gamepad1.dpad_up) {
            target += 250;
        }
        else if (gamepad1.dpad_down) {
            target -= 250;
        }



        //intake wrist
        if (gamepad1.right_bumper) {
            iServoLeft.setPosition(0); //0  is down
            iServoRight.setPosition(1); // 1 down
        } else if (gamepad1.left_bumper) {
            iServoLeft.setPosition(1); // 1 is up
            iServoRight.setPosition(0); // 0 is up
        }

        // intake cr servos
        if (gamepad1.x) {
            iCRServoLeft.setPower(1.0); // Run forward
            iCRServoRight.setPower(-1.0);
        } else if (gamepad1.y) {
            iCRServoLeft.setPower(-1.0); // Run backward
            iCRServoRight.setPower(1.0);

        } else {
            iCRServoLeft.setPower(0);
            iCRServoRight.setPower(0);// Stop
        }


        //outtale servos
        if (gamepad1.a) {
            oCRServoLeft.setPower(1);
            oCRServoRight.setPower(-1);
        } else if (gamepad1.b) {
            oCRServoLeft.setPower(-1);
            oCRServoRight.setPower(1);
        } else {
            oCRServoLeft.setPower(0);
            oCRServoRight.setPower(0);
        }


        if (gamepad1.dpad_left) {
            Actuator.setPower(1.0); // Extend
        } else if (gamepad1.dpad_right) {
            Actuator.setPower(-1.0); // Retract
        } else {
            Actuator.setPower(0); // Stop
        }




        target = Math.max(2500, Math.min(target, 65000));

        // PID output
        double pid = controller.calculate(slidesPos, target);

        // gravity compensation + PID correction
        double power = pid + kg;

        // Apply power to both slides (consider reversed direction for one motor)
        leftLinearSlide.setPower(power);
        rightLinearSlide.setPower(power);

        // telemetry for debugging
        telemetry.addData("Left Slide Position", leftSlidePos);
        telemetry.addData("Right Slide Position", rightSlidePos);
        telemetry.addData("Target Position", target);
        telemetry.addData("Slide Position (Avg)", slidesPos);
        telemetry.addData("PID Output", pid);
        telemetry.addData("Motor Power", power);
        telemetry.update();
    }
}

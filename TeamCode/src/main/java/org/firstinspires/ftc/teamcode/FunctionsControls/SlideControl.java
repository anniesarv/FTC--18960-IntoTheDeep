
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

@Config
@TeleOp
public class SlideControl extends OpMode {
    private PIDController slideController;

    public static double p = 0.001, i = 0, d =0.00001 ;
    public static double kg = 0.1; // gravity compensation
    public static int target = 1000; // target position in encoder ticks

    private final double ticks_in_inches = 66275 / 52.0; // Full extension ticks per inch

    private DcMotorEx leftLinearSlide;
    private DcMotorEx rightLinearSlide;

    @Override
    public void init() {
        slideController = new PIDController(p, i, d);
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

    @Override
    public void loop() {
        slideController.setPID(p, i, d); // Update PID values from dashboard

        int leftSlidePos = leftLinearSlide.getCurrentPosition(); // Negate for correct direction
        int rightSlidePos = -rightLinearSlide.getCurrentPosition();

        // Average slide positions
        int slidesPos = (leftSlidePos );//+ rightSlidePos) / 2;

        //dpad input i hope it works
        if (gamepad1.dpad_up) {
            target += 300;
        }
        else if (gamepad1.dpad_down) {
            target -= 300;
        }

        target = Math.max(0, Math.min(target, 60000));

        // PID output
        double pid = slideController.calculate(slidesPos, target);

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

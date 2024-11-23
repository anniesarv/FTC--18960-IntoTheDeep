package org.firstinspires.ftc.teamcode.FTClibSubsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@Autonomous
public class CookyPid extends OpMode {
    private PIDController controller;

    public static double p = 0.001, i = 0, d =0.00001 ;
    public static double kg = 0.1; // gravity compensation
    public static int target = 20000; // target position in encoder ticks

    private final double ticks_in_inches = 66275 / 52.0; // Full extension ticks per inch

    private DcMotorEx leftLinearSlide;
    private DcMotorEx rightLinearSlide;

    @Override
    public void init() {
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

    @Override
    public void loop() {
        controller.setPID(p, i, d); // Update PID values from dashboard

        int leftSlidePos = leftLinearSlide.getCurrentPosition(); // Negate for correct direction
       int rightSlidePos = -rightLinearSlide.getCurrentPosition();

        // Average slide positions
        int slidesPos = (leftSlidePos );//+ rightSlidePos) / 2;

        // PID output
        double pid = controller.calculate(slidesPos, target);

        // Gravity compensation + PID correction
        double power = pid + kg;

        // Apply power to both slides (consider reversed direction for one motor)
        leftLinearSlide.setPower(power);
        rightLinearSlide.setPower(power);

        // Telemetry for debugging
        telemetry.addData("Left Slide Position", leftSlidePos);
        telemetry.addData("Right Slide Position", rightSlidePos);
        telemetry.addData("Target Position", target);
        telemetry.addData("Slide Position (Avg)", slidesPos);
        telemetry.addData("PID Output", pid);
        telemetry.addData("Motor Power", power);
        telemetry.update();
    }
}

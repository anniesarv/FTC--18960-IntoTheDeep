package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class AutoSlideSubsystem {
    private final PIDController controller;
    private final DcMotorEx leftLinearSlide;
    private final DcMotorEx rightLinearSlide;

    // Adjustable parameters via dashboard
    public static double p = 0.001, i = 0, d = 0.00001;
    public static double kg = 0.1; // Gravity compensation
    private int target = 10; // Default target position in encoder ticks

    public AutoSlideSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry) {
        // Initialize PID controller
        controller = new PIDController(p, i, d);

        // Initialize motors
        leftLinearSlide = hardwareMap.get(DcMotorEx.class, "leftLinearSlide");
        rightLinearSlide = hardwareMap.get(DcMotorEx.class, "rightLinearSlide");

        // Configure motors
        leftLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLinearSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightLinearSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftLinearSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightLinearSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        leftLinearSlide.setDirection(DcMotorEx.Direction.REVERSE);
        rightLinearSlide.setDirection(DcMotor.Direction.FORWARD);

        // Optional: Pass telemetry for debugging
        telemetry.addData("LinearSlideSubsystem", "Initialized");
        telemetry.update();
    }

    public void setTarget(int targetPosition) {
        this.target = targetPosition;
    }

    public void update() {
        // Update PID gains
        controller.setPID(p, i, d);

        int leftSlidePos = leftLinearSlide.getCurrentPosition();
        int rightSlidePos = -rightLinearSlide.getCurrentPosition();

        // Average slide positions
        int slidesPos = leftSlidePos; // Can add more averaging if needed

        // Calculate PID correction
        double pid = controller.calculate(slidesPos, target);

        // Gravity compensation
        double power = pid + kg;

        // Apply power to both slides
        leftLinearSlide.setPower(power);
        rightLinearSlide.setPower(power);
    }
}

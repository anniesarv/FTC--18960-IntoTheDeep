package org.firstinspires.ftc.teamcode.FunctionsControls;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SlideSubsystem {
    private DcMotorEx leftLinearSlide, rightLinearSlide;
    private PIDController controller;
    private static final double KG = 0.1; // Gravity compensation
    public static double p = 0.001, i = 0, d = 0.00001;
    private final double ticks_in_inches = 66275 / 52.0; // Full extension ticks per inch

    public int target = 0;  // Initial target position

    public SlideSubsystem(HardwareMap hardwareMap, double p, double i, double d) {
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

        controller = new PIDController(p, i, d);

    }

    public void updatePID(double p, double i, double d) {
        controller.setPID(p, i, d);
    }

    public void setTarget(int target) {
        this.target = Math.max(0, Math.min(target, 65000)); // Clamp target to valid range
    }

    // New method to control slides using dpad buttons
    public void controlWithDpad(boolean dpadUp, boolean dpadDown) {
        // Adjust the target based on dpad input
        if (dpadUp) {
            setTarget(target + 1000);  // Increase target by 1000 (or any value you prefer)
        } else if (dpadDown) {
            setTarget(target - 1000);  // Decrease target by 1000 (or any value you prefer)
        }

    }

    // Updated controlSlides method to use the target
    public void controlSlides() {
        int leftSlidePos = leftLinearSlide.getCurrentPosition();
        int rightSlidePos = rightLinearSlide.getCurrentPosition();
        int slidesPos = (leftSlidePos + rightSlidePos) / 2; // Average position

        double pid = controller.calculate(slidesPos, target);
        double power = pid + KG;

        leftLinearSlide.setPower(power);
        rightLinearSlide.setPower(power);
    }

    public int getTarget() {
        return target;
    }
}

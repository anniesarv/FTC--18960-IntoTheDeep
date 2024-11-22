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
import com.qualcomm.robotcore.hardware.HardwareMap;


public class SlideSubsystem {
    private DcMotorEx leftLinearSlide, rightLinearSlide;
    private PIDController controller;
    private static final double KG = 0.1; // Gravity compensation
    private int target = 1000;

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





    public void controlSlides() {
        int leftSlidePos = leftLinearSlide.getCurrentPosition();
        int rightSlidePos = rightLinearSlide.getCurrentPosition();
        int slidesPos = leftSlidePos + rightSlidePos; // Average or adjust if necessary

        double pid = controller.calculate(slidesPos, target);
        double power = pid + KG;

        leftLinearSlide.setPower(power);
        rightLinearSlide.setPower(power);
    }

    public int getTarget() {
        return target;
    }
}




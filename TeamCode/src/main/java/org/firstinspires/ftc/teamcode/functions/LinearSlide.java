package org.firstinspires.ftc.teamcode.functions;

import org.firstinspires.ftc.teamcode.functions.ProportionalControl;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LinearSlide {
    public DcMotorEx leftLinearSlide;
    public DcMotorEx rightLinearSlide;

    private ProportionalControl slideControl;

    private static final int MAX_POSITION = 5000; // Example max encoder position
    private static final int MIN_POSITION = 0;    // Example min encoder position

    public LinearSlide(HardwareMap hardwareMap) {
        leftLinearSlide = hardwareMap.get(DcMotorEx.class, "slideMotorLeft");
        rightLinearSlide = hardwareMap.get(DcMotorEx.class, "slideMotorRight");

        leftLinearSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightLinearSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftLinearSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightLinearSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        leftLinearSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightLinearSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        slideControl = new ProportionalControl(
                0.01,  // Proportional gain
                5.0,   // Acceleration limit
                1.0,   // Max output power
                10,    // Tolerance (encoder ticks)
                5,     // Deadband (encoder ticks)
                false  // Not circular
        );
    }

    public void extendSlides() {
        setTargetPosition(MAX_POSITION);
    }

    public void retractSlides() {
        setTargetPosition(MIN_POSITION);
    }

    private void setTargetPosition(int targetPosition) {
        targetPosition = Math.min(Math.max(targetPosition, MIN_POSITION), MAX_POSITION);
        slideControl.reset(targetPosition);
    }

    public void update() {
        int currentPositionLeft = leftLinearSlide.getCurrentPosition();
        int currentPositionRight = rightLinearSlide.getCurrentPosition();
        int averagePosition = (currentPositionLeft + currentPositionRight) / 2;

        double power = slideControl.getOutput(averagePosition);

        if ((averagePosition >= MAX_POSITION && power > 0) ||
                (averagePosition <= MIN_POSITION && power < 0)) {
            power = 0;
        }

        leftLinearSlide.setPower(power);
        rightLinearSlide.setPower(power);
    }

    public boolean isInPosition() {
        return slideControl.inPosition();
    }
}

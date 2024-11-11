package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class ServoTest extends LinearOpMode {

    private Servo iServoLeft;
    private Servo iServoRight;

    private CRServo iCRServoLeft;
    private CRServo iCRServoRight;

    @Override
    public void runOpMode() {
        // Initialize the servos
        iServoLeft = hardwareMap.get(Servo.class, "iServoLeft");
        iServoRight = hardwareMap.get(Servo.class, "iServoRight");
        iCRServoLeft = hardwareMap.get(Servo.class "iCRServoLeft");

        // Wait for the start button to be pressed
        waitForStart();

        // Set both servos to max speed (1) once the OpMode starts
        if (opModeIsActive()) {
            iServoLeft.setPosition(1);
            iServoRight.setPosition(0);
        }

        // Keep the servos running until the OpMode is stopped
        while (opModeIsActive()) {
            // Add telemetry to monitor if needed
            telemetry.addData("iServoLeft Power", iServoLeft.getPosition());
            telemetry.addData("iServoRight Power", iServoRight.getPosition());
            telemetry.update();
        }


        //TEST FOR GITHUB
        //second commit test
        //branch testgi
    }
}

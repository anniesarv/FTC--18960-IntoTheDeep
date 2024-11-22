package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Servo Test", group = "Testing")
public class ServoTest extends LinearOpMode {

    private Servo iServoLeft;
    private Servo iServoRight;
    private CRServo iCRServoLeft;
    private CRServo iCRServoRight;
    private CRServo oCRServoRight;
    private CRServo oCRServoLeft;

    @Override
    public void runOpMode() {
        // Initialize the servos
        iServoLeft = hardwareMap.get(Servo.class, "iServoLeft");
        iServoRight = hardwareMap.get(Servo.class, "iServoRight");
        iCRServoLeft = hardwareMap.get(CRServo.class, "iCRServoLeft");
        iCRServoRight = hardwareMap.get(CRServo.class, "iCRServoRight");
        oCRServoLeft = hardwareMap.get(CRServo.class, "oCRServoLeft");
        oCRServoRight = hardwareMap.get(CRServo.class, "oCRServoRight");


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            // Control servo positions with gamepad buttons
            if (gamepad1.dpad_down) {
                iServoLeft.setPosition(0); //0  is down
                iServoRight.setPosition(1); // 1 down
            } else if (gamepad1.dpad_up) {
                iServoLeft.setPosition(1); // 1 is up
                iServoRight.setPosition(0); // 0 is up
            }





          // Example of CRServo usage(if applicable)
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










            // Telemetry to monitor positions and power
           // telemetry.addData("iServoLeft Position", iServoLeft.getPosition());
            telemetry.addData("iServoRight Position", iServoRight.getPosition());
           // telemetry.addData("iCRServoLeft Power", iCRServoLeft.getPower());
            telemetry.update();
        }
    }
}

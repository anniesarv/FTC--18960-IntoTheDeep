package org.firstinspires.ftc.teamcode.FunctionsControls;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


public class ServoSubsystem {
    private Servo iServoLeft, iServoRight;
    private CRServo iCRServoLeft, iCRServoRight, oCRServoLeft, oCRServoRight;
    private ElapsedTime timer = new ElapsedTime(); // Add this to the class


    public ServoSubsystem(HardwareMap hardwareMap) {
        iServoLeft = hardwareMap.get(Servo.class, "iServoLeft");
        iServoRight = hardwareMap.get(Servo.class, "iServoRight");
        iCRServoLeft = hardwareMap.get(CRServo.class, "iCRServoLeft");
        iCRServoRight = hardwareMap.get(CRServo.class, "iCRServoRight");
        oCRServoLeft = hardwareMap.get(CRServo.class, "oCRServoLeft");
        oCRServoRight = hardwareMap.get(CRServo.class, "oCRServoRight");
    }

    public void setIntakeWristPosition(boolean rightBumper, boolean leftBumper) {
        if (rightBumper) {
            iServoLeft.setPosition(0.1);
            iServoRight.setPosition(0.9);
        } else if (leftBumper) {
            iServoLeft.setPosition(1);
            iServoRight.setPosition(0);
        }
    }

    public void controlIntake(boolean xPressed, boolean yPressed) {
        if (xPressed) {
            iCRServoLeft.setPower(1.0);
            iCRServoRight.setPower(-1.0);
        } else if (yPressed) {
            iCRServoLeft.setPower(-1.0);
            iCRServoRight.setPower(1.0);
        } else {
            iCRServoLeft.setPower(0);
            iCRServoRight.setPower(0);
        }
    }

    public void controlOuttake(boolean aPressed, boolean bPressed) {


        if (aPressed) {
            oCRServoLeft.setPower(1);
            oCRServoRight.setPower(-1);
        } else if (bPressed) {
            oCRServoLeft.setPower(-1);
            oCRServoRight.setPower(1);


        } else {
            oCRServoLeft.setPower(0);
            oCRServoRight.setPower(0);
        }
    }


    private boolean isTransferStarted = false; // Tracks if the transfer has started

    public void servoTransfer(boolean dpaddown) {
        if (dpaddown) {
            if (!isTransferStarted) {
                // Start running CR servos and reset the timer only once
                oCRServoLeft.setPower(-1);
                oCRServoRight.setPower(1);
                timer.reset();
                isTransferStarted = true;
            }

            // Wait for 1 second
            if (timer.seconds() >= 1.0) {
                // Stop CR servos and move the positional servos
                oCRServoLeft.setPower(0);
                oCRServoRight.setPower(0);
                iServoLeft.setPosition(0.1);
                iServoRight.setPosition(0.9);

                // Reset the state to allow another transfer
                isTransferStarted = false;
            }
        } else {
            // Stop CR servos and reset the state if dpaddown isreleased
            oCRServoLeft.setPower(0);
            oCRServoRight.setPower(0);
            isTransferStarted = false;
        }
    }


    public void wristTransfer(boolean dpadup) {
        if (dpadup) {
            // Start running CR servos
            iServoLeft.setPosition(1);
            iServoRight.setPosition(0);
        }
    }


}

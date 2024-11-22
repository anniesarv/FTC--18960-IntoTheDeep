
package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Actuator Test", group = "Testing")
public class ActuatorTest extends LinearOpMode {
    private DcMotor Actuator;

    @Override
    public void runOpMode() {
        // Initialize the linear actuator motor
        Actuator = hardwareMap.get(DcMotor.class, "axial");

        Actuator.setDirection(DcMotorSimple.Direction.FORWARD);
        Actuator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start
        waitForStart();

        while (opModeIsActive()) {
            // Control actuator with gamepad buttons
            if (gamepad1.a) {
                Actuator.setPower(1.0); // Extend
            } else if (gamepad1.b) {
                Actuator.setPower(-1.0); // Retract
            } else {
                Actuator.setPower(0); // Stop
            }

            // Telemetry for debugging
            telemetry.addData("Actuator Power", Actuator.getPower());
            telemetry.update();
        }
    }
}



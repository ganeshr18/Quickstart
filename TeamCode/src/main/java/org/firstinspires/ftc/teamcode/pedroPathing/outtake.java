package org.firstinspires.ftc.teamcode.pedroPathing;
// This line declares the package location of your code (inside "teamcode").
// It helps organize and access your OpMode from the FTC SDK.

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
// These imports bring in the FTC SDK classes you'll need to create a Linear OpMode
// and access the DC motor hardware.

@TeleOp(name="Outtake Control", group="Linear OpMode")
// This annotation registers your program as a TeleOp mode on the Driver Station app.
// "Outtake Control" is the name that will appear in the list.

public class outtake extends LinearOpMode {
// This defines your main class, which extends LinearOpMode.
// LinearOpMode runs your robot code in a continuous linear loop.

    private DcMotor outtake1 = null;
    // This creates a variable for your DC motor. It will later link to the hardware configuration.

    @Override
    public void runOpMode() {
        // This method contains all the code that runs when your OpMode starts.

        // Initialize the motor from the hardware configuration file on the Control Hub
        outtake1 = hardwareMap.get(DcMotor.class, "outtake");
        // "outtake" must match the name of your motor in the FTC Robot Controller configuration.

        // Reverse the motor’s direction so it spins the other way
        // (change this back to FORWARD if you ever need to undo the reversal)
        outtake1.setDirection(DcMotor.Direction.FORWARD);

        // Display a message on the Driver Station to show the robot is ready
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the driver to press PLAY on the Driver Station
        waitForStart();

        // Run this loop continuously until STOP is pressed
        while (opModeIsActive()) {

            // Read trigger values from the gamepad
            double outPower = gamepad1.right_trigger;  // Right trigger spins motor forward (outtake)
            double inPower = gamepad1.left_trigger;    // Left trigger spins motor backward (intake)

            // Subtract left trigger value from right trigger value
            // so pressing both gives a balanced power (0)
            double power = outPower - inPower;

            // Apply the calculated power to the outtake motor
            outtake1.setPower(power);

            // Display motor power on the Driver Station for debugging
            telemetry.addData("Outtake Power", "%.2f", power);
            telemetry.update();
        }
    }
}



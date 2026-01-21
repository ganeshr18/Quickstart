package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Drive extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        DcMotor front_left_drive = hardwareMap.dcMotor.get("front_left_drive");
        DcMotor back_left_drive = hardwareMap.dcMotor.get("back_left_drive");
        DcMotor front_right_drive = hardwareMap.dcMotor.get("front_right_drive");
        DcMotor back_right_drive = hardwareMap.dcMotor.get("back_right_drive");
        DcMotor Intake = hardwareMap.dcMotor.get("Intake");
        DcMotor Outtake = hardwareMap.dcMotor.get("Outtake");
        DcMotor flywheel = hardwareMap.dcMotor.get("flywheel");

        Limelight3A cam = hardwareMap.get(Limelight3A.class, "limelight");


        // Reverse the right side motors
        front_left_drive.setDirection(DcMotorSimple.Direction.FORWARD);
        back_left_drive.setDirection(DcMotorSimple.Direction.FORWARD);
        front_right_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right_drive.setDirection(DcMotorSimple.Direction.FORWARD);


        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Toggle state variables
        boolean intakeRunning = true;
        boolean outtakeRunning = false;
        boolean lastRightTriggerPressed = false;
        boolean lastLeftTriggerPressed = false;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // ========== OMNIDIRECTIONAL DRIVE CONTROLS ==========
            // Left joystick controls direction of movement
            double y = -gamepad1.left_stick_y; // Forward/backward
            double x = gamepad1.left_stick_x; // Strafe left/right

            // Right joystick controls rotation
            double rx = gamepad1.right_stick_x * 0.5;

            // Add deadzone to prevent stick drift
            double deadzone = 0.05;
            if (Math.abs(y) < deadzone) y = 0;
            if (Math.abs(x) < deadzone) x = 0;
            if (Math.abs(rx) < deadzone) rx = 0;


            // Calculate motor powers for mecanum drive
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            front_left_drive.setPower(frontLeftPower);
            back_left_drive.setPower(backLeftPower);
            front_right_drive.setPower(frontRightPower);
            back_right_drive.setPower(backRightPower);

            // ========== INTAKE TOGGLE (FLYWHEEL AT MAX POWER) ==========
            boolean leftTriggerPressed = gamepad1.left_trigger > 0.5;
            if (leftTriggerPressed && !lastLeftTriggerPressed) {
                intakeRunning = !intakeRunning;
            }
            lastLeftTriggerPressed = leftTriggerPressed;

            if (intakeRunning) {
                Intake.setPower(0.6); // Max power for flywheel
            } else {
                Intake.setPower(0);
            }

            // ========== OUTTAKE TOGGLE (MAX POWER) ==========
            boolean rightTriggerPressed = gamepad1.right_trigger > 0.1;
            if (rightTriggerPressed && !lastRightTriggerPressed) {
                outtakeRunning = !outtakeRunning;
            }
            lastRightTriggerPressed = rightTriggerPressed;

            if (gamepad1.right_trigger > 0.1) {
                Outtake.setPower(1.0); // Max power
            } else {
                Outtake.setPower(0);
            }

            if(gamepad1.left_bumper){
                Intake.setPower(-0.8);
            }

            // ========== GATE SERVO CONTROL ==========
            if (gamepad1.right_bumper){
                flywheel.setPower(0.7);
            }
            else{
                flywheel.setPower(0.58);
            }
            // ========== TELEMETRY ==========
            telemetry.addData("Intake Running", intakeRunning);
            telemetry.addData("Outtake Running", outtakeRunning);
            telemetry.update();
        }
    }
}
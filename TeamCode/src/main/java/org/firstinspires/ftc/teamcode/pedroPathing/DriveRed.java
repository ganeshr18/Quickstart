package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class DriveRed extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        double F = 32767/5400;
        double P = 201.0;

        DcMotorEx flywheelMotor = hardwareMap.get(DcMotorEx.class,"flywheel");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        DcMotor front_left_drive = hardwareMap.dcMotor.get("front_left_drive");
        DcMotor back_left_drive = hardwareMap.dcMotor.get("back_left_drive");
        DcMotor front_right_drive = hardwareMap.dcMotor.get("front_right_drive");
        DcMotor back_right_drive = hardwareMap.dcMotor.get("back_right_drive");
        DcMotor Intake = hardwareMap.dcMotor.get("Intake");
        DcMotor Outtake = hardwareMap.dcMotor.get("Outtake");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);

        limelight.start();

        // Reverse the right side motors
        front_left_drive.setDirection(DcMotorSimple.Direction.FORWARD);
        back_left_drive.setDirection(DcMotorSimple.Direction.FORWARD);
        front_right_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right_drive.setDirection(DcMotorSimple.Direction.FORWARD);

        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Toggle state variables
        boolean intakeRunning = true;
        boolean outtakeRunning = false;
        boolean lastRightTriggerPressed = false;
        boolean lastLeftTriggerPressed = false;
        boolean align = false;

        double offset = 0;
        double area = 0;
        double flywheelpower = 1400;
        double conversionrate = 2520;

        double alignTurnSpeed = 0.17;
        double alignSensitivity = 0.5;
        boolean llworks = false;
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Get Limelight result once per loop
            LLResult llResult = limelight.getLatestResult();
            llworks = llResult != null && llResult.isValid();

            if (llworks) {
                offset = llResult.getTx();
                if (llResult.getTa() != 0) {
                    area = llResult.getTa();
                }

                flywheelpower = (-0.0618121 * Math.pow(area, 4))+(0.175134 * Math.pow(area, 3))+(0.0112728 * Math.pow(area, 2))+(-0.365695 * area)+0.792917;
                if(area<0.5){
                    alignSensitivity=0.3;
                    conversionrate=2460;
                }
                else{
                    alignSensitivity=0.5;
                    conversionrate=2520;
                }
                flywheelpower*=conversionrate;
            }

            // ========== LIMELIGHT AUTO-ALIGN ==========
            if (gamepad1.a) {
                align = true;
            }


            if (align && llworks) {
                if (Math.abs(offset) > alignSensitivity) {
                    if (offset < 0) {
                        // Turn left
                        front_left_drive.setPower(-1 * alignTurnSpeed);
                        back_left_drive.setPower(-1 * alignTurnSpeed);
                        front_right_drive.setPower(alignTurnSpeed);
                        back_right_drive.setPower(alignTurnSpeed);
                    } else {
                        // Turn right
                        front_left_drive.setPower(alignTurnSpeed);
                        back_left_drive.setPower(alignTurnSpeed);
                        front_right_drive.setPower(-1 * alignTurnSpeed);
                        back_right_drive.setPower(-1 * alignTurnSpeed);
                    }
                } else {
                    // Aligned! Stop motors
                    align = false;
                    front_left_drive.setPower(0);
                    back_left_drive.setPower(0);
                    front_right_drive.setPower(0);
                    back_right_drive.setPower(0);
                    telemetry.addData("Status", "ALIGNED!");
                }
            } else {
                // ========== OMNIDIRECTIONAL DRIVE CONTROLS ==========
                // Left joystick controls direction of movement
                double y = -gamepad1.left_stick_y; // Forward/backward
                double x = gamepad1.left_stick_x; // Strafe left/right

                // Right joystick controls rotation
                double rx = gamepad1.right_stick_x * 0.7;

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

                // Normal driving
                front_left_drive.setPower(frontLeftPower);
                back_left_drive.setPower(backLeftPower);
                front_right_drive.setPower(frontRightPower);
                back_right_drive.setPower(backRightPower);
            }

            // ========== INTAKE TOGGLE (FLYWHEEL AT MAX POWER) ==========
            boolean leftTriggerPressed = gamepad1.left_trigger > 0.5;
            if (leftTriggerPressed && !lastLeftTriggerPressed) {
                intakeRunning = !intakeRunning;
            }
            lastLeftTriggerPressed = leftTriggerPressed;

            if (intakeRunning) {
                Intake.setPower(1); // Max power for flywheel
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

            if (gamepad1.left_bumper) {
                Intake.setPower(-0.8);
            }

            // ========== FLYWHEEL CONTROL ==========
            pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
            flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

            // set velocity
            flywheelMotor.setVelocity(flywheelpower);

            // ========== TELEMETRY ==========
            telemetry.addData("A Button", gamepad1.a ? "PRESSED" : "not pressed");
            telemetry.addData("Align Mode", align);
            telemetry.addData("Limelight Works", llworks);
            telemetry.addData("Intake Running", intakeRunning);
            telemetry.addData("Outtake Running", outtakeRunning);
            telemetry.addData("Flywheel Speed:", flywheelpower);
            telemetry.addData("Area:", area);
            telemetry.update();
        }
    }
}
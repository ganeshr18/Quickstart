package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class Drive_but_its_made_by_AI extends LinearOpMode {

    // PID Controller for Launcher Velocity Control
    class LauncherPIDController {
        private double Kp, Ki, Kd, Kf;
        private double integralSum = 0;
        private double lastError = 0;
        private ElapsedTime timer = new ElapsedTime();
        private double maxIntegral = 0.3;

        public LauncherPIDController(double kp, double ki, double kd, double kf) {
            this.Kp = kp;
            this.Ki = ki;
            this.Kd = kd;
            this.Kf = kf; // Feedforward for velocity control
        }

        public double calculate(double targetVelocity, double currentVelocity) {
            double error = targetVelocity - currentVelocity;
            double deltaTime = timer.seconds();

            if (deltaTime == 0) {
                return 0;
            }

            timer.reset();

            // Proportional term
            double p = Kp * error;

            // Integral term with anti-windup clamping
            integralSum += error * deltaTime;
            integralSum = Math.max(-maxIntegral, Math.min(maxIntegral, integralSum));
            double i = Ki * integralSum;

            // Derivative term
            double derivative = (error - lastError) / deltaTime;
            double d = Kd * derivative;

            // Feedforward term (helps maintain constant velocity)
            double f = Kf * targetVelocity;

            lastError = error;

            return p + i + d + f;
        }

        public void reset() {
            integralSum = 0;
            lastError = 0;
            timer.reset();
        }
    }

    // Target velocity for launcher in ticks per second
    private static final double LAUNCHER_TARGET_VELOCITY = 2000; // Adjust this value for your robot

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        DcMotor front_left_drive = hardwareMap.dcMotor.get("front_left_drive");
        DcMotor back_left_drive = hardwareMap.dcMotor.get("back_left_drive");
        DcMotor front_right_drive = hardwareMap.dcMotor.get("front_right_drive");
        DcMotor back_right_drive = hardwareMap.dcMotor.get("back_right_drive");
        DcMotor Intake = hardwareMap.dcMotor.get("Intake");
        DcMotor Outtake = hardwareMap.dcMotor.get("Outtake");
        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "flywheel"); // Use DcMotorEx for velocity


        // Reverse the right side motors
        front_left_drive.setDirection(DcMotorSimple.Direction.FORWARD);
        back_left_drive.setDirection(DcMotorSimple.Direction.FORWARD);
        front_right_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right_drive.setDirection(DcMotorSimple.Direction.FORWARD);

        // Configure flywheel motor
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // Float for flywheel

        // Create PID controller for launcher velocity
        // Tune these values: Kp, Ki, Kd, Kf
        LauncherPIDController launcherPID = new LauncherPIDController(0.0003, 0.00001, 0.00005, 0.00015);

        // Toggle state variables
        boolean intakeRunning = false;
        boolean outtakeRunning = false;
        boolean flywheelRunning = false;
        boolean lastRightTriggerPressed = false;
        boolean lastLeftTriggerPressed = false;
        boolean lastAButton = false;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // ========== OMNIDIRECTIONAL DRIVE CONTROLS ==========
            // Left joystick controls direction of movement
            double y = -gamepad1.left_stick_y; // Forward/backward
            double x = gamepad1.left_stick_x; // Strafe left/right

            // Right joystick controls rotation
            double rx = gamepad1.right_stick_x;

            // Calculate motor powers for mecanum drive
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y - x + rx) / denominator;
            double backLeftPower = (y - x - rx) / denominator;
            double frontRightPower = (y + x - rx) / denominator;
            double backRightPower = (y + x + rx) / denominator;

            front_left_drive.setPower(frontLeftPower);
            back_left_drive.setPower(backLeftPower);
            front_right_drive.setPower(frontRightPower);
            back_right_drive.setPower(backRightPower);

            // ========== INTAKE TOGGLE ==========
            boolean leftTriggerPressed = gamepad1.left_trigger > 0.1;
            if (leftTriggerPressed && !lastLeftTriggerPressed) {
                intakeRunning = !intakeRunning;
            }
            lastLeftTriggerPressed = leftTriggerPressed;

            if (intakeRunning) {
                Intake.setPower(0.6);
            } else {
                Intake.setPower(0);
            }

            // ========== OUTTAKE TOGGLE ==========
            boolean rightTriggerPressed = gamepad1.right_trigger > 0.1;
            if (rightTriggerPressed && !lastRightTriggerPressed) {
                outtakeRunning = !outtakeRunning;
            }
            lastRightTriggerPressed = rightTriggerPressed;

            if (outtakeRunning) {
                Outtake.setPower(1.0);
            } else {
                Outtake.setPower(0);
            }

            // Intake reverse
            if(gamepad1.left_bumper){
                Intake.setPower(-0.6);
            }

            // ========== FLYWHEEL PID CONTROL ==========
            // Toggle flywheel with A button
            if (gamepad1.a && !lastAButton) {
                flywheelRunning = !flywheelRunning;
                if (flywheelRunning) {
                    launcherPID.reset(); // Reset PID when starting
                }
            }
            lastAButton = gamepad1.a;

            if (flywheelRunning) {
                // Get current velocity in ticks per second
                double currentVelocity = flywheel.getVelocity();

                // Calculate PID output
                double pidOutput = launcherPID.calculate(LAUNCHER_TARGET_VELOCITY, currentVelocity);

                // Clamp output to motor power limits
                pidOutput = Math.max(0, Math.min(0.6, pidOutput));

                flywheel.setPower(pidOutput);
            } else {
                flywheel.setPower(0);
            }

            // ========== GATE SERVO CONTROL ==========


            // ========== TELEMETRY ==========
            telemetry.addData("Intake Running", intakeRunning);
            telemetry.addData("Outtake Running", outtakeRunning);
            telemetry.addData("Flywheel Running", flywheelRunning);
            if (flywheelRunning) {
                telemetry.addData("Flywheel Velocity", "%.0f / %.0f", flywheel.getVelocity(), LAUNCHER_TARGET_VELOCITY);
                telemetry.addData("Flywheel Error", LAUNCHER_TARGET_VELOCITY - flywheel.getVelocity());
                telemetry.addData("Flywheel Power", "%.2f", flywheel.getPower());
            }
            telemetry.update();
        }
    }
}

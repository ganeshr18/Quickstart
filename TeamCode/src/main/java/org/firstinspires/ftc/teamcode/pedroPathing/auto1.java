package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class auto1 extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)

    private Timer pathTimer, actionTimer, opmodeTimer;
    private Paths paths; // Paths defined in the Paths class

    // Declare motors but don't initialize yet
    DcMotor flyWheel;
    DcMotor outtake;
    DcMotor intake;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize hardware INSIDE init()
        flyWheel = hardwareMap.dcMotor.get("Outtake");
        outtake = hardwareMap.dcMotor.get("flywheel");
        intake = hardwareMap.dcMotor.get("Intake");

        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56.000, 8.000, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        // Initialize timers
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();

        pathState = 0; // Initialize path state

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        outtake.setPower(0.7);
        autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }






    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.000, 8.000),

                                    new Pose(60.000, 15.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(107))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(60.000, 15.000),

                                    new Pose(4.000, 11.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(107), Math.toRadians(180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(4.000, 11.000),

                                    new Pose(60.000, 14.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(60.000, 14.000),

                                    new Pose(15.619, 17.221)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(70))

                    .build();
        }
    }






    public void autonomousPathUpdate() {
        switch (pathState){
            case 0:
                if (pathTimer.getElapsedTime() > 4000) {
                    follower.followPath(paths.Path1);
                    setPathState(1);
                }
                break; // IMPORTANT!

            case 1:
                if(!follower.isBusy()) {
                    setPathState(2);
                }
                break; // IMPORTANT!
            case 2:
                if (pathTimer.getElapsedTime() > 4000){
                    intake.setPower(1.0);
                    flyWheel.setPower(0.0);
                    follower.followPath(paths.Path2, true);
                    setPathState(3);
                }
                else{
                    intake.setPower(1.0);
                    flyWheel.setPower(1.0);
                }
                break;

            case 3:
                if(!follower.isBusy() && pathTimer.getElapsedTime() > 4000) {
                    follower.followPath(paths.Path3, true);
                    setPathState(4);
                }
                else{
                    intake.setPower(1.0);
                }
                break; // IMPORTANT!
            case 4:
                if(!follower.isBusy()) {
                    setPathState(5);
                }
                break; // IMPORTANT!
            case 5:
                if (pathTimer.getElapsedTime() > 4000){
                    intake.setPower(1.0);
                    flyWheel.setPower(0.0);
                    follower.followPath(paths.Path4, true);
                    setPathState(-1);
                }
                else{
                    intake.setPower(1.0);
                    flyWheel.setPower(1.0);
                }
                break;

        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}
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


@Autonomous(name = "Red-Close-To-You", group = "Autonomous")
@Configurable // Panels
public class Red_close extends OpMode {
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
        follower.setStartingPose(new Pose(110, 145, Math.toRadians(0)));

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
        outtake.setPower(0.52);
        intake.setPower(1.0);
        autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }













    public static class Paths {
        public PathChain shoot1;
        public PathChain pickup1;
        public PathChain shoot2;
        public PathChain pickup21;
        public PathChain pickup22;
        public PathChain shoot3;
        public PathChain gate;

        public Paths(Follower follower) {
            shoot1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(110.000, 145.000),

                                    new Pose(84.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50))

                    .build();

            pickup1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(84.000, 84.000),

                                    new Pose(130.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(0))

                    .build();

            shoot2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(130.000, 84.000),

                                    new Pose(84.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50))

                    .build();

            pickup21 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(84.000, 84.000),

                                    new Pose(84.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(0))

                    .build();

            pickup22 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(84.000, 60.000),

                                    new Pose(133.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            shoot3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(133.000, 60.000),

                                    new Pose(84.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50))

                    .build();

            gate = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(84.000, 84.000),

                                    new Pose(137.000, 70.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(90))

                    .build();
        }
    }










    public void autonomousPathUpdate() {
        switch (pathState){
            case 0:
                if (pathTimer.getElapsedTime() > 1000) {
                    follower.followPath(paths.shoot1, 0.6, false);
                    setPathState(1);
                }
                break; // IMPORTANT!

            case 1:
                if(!follower.isBusy()) {
                    setPathState(2);
                }
                break; // IMPORTANT!
            case 2:
                if (pathTimer.getElapsedTime() > 2000){

                    flyWheel.setPower(0.0);
                    follower.followPath(paths.pickup1, 0.6, true);
                    setPathState(3);
                }
                else{
                    flyWheel.setPower(1.0);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(paths.shoot2, 0.6, true);
                    setPathState(4);
                }
                break; // IMPORTANT!
            case 4:
                if(!follower.isBusy()) {
                    setPathState(5);
                }
                break; // IMPORTANT!
            case 5:
                if (pathTimer.getElapsedTime() > 3000){
                    flyWheel.setPower(0.0);
                    follower.followPath(paths.pickup21, 0.6, true);
                    setPathState(6);
                }
                else{
                    flyWheel.setPower(1.0);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(paths.pickup22, 0.6, true);
                    setPathState(7);
                }
                break; // IMPORTANT!
            case 7:
                if(!follower.isBusy()) {
                    follower.followPath(paths.shoot3, 0.6, true);
                    setPathState(8);
                }
                break; // IMPORTANT!
            case 8:
                if(!follower.isBusy()) {
                    setPathState(9);
                }
                break; // IMPORTANT!
            case 9:
                if (pathTimer.getElapsedTime() > 3000){
                    flyWheel.setPower(0.0);
                    follower.followPath(paths.gate, 0.6, true);
                    setPathState(-1);
                }
                else{
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
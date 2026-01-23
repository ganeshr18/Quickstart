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


@Autonomous(name = "blue 12 ball", group = "Autonomous")
@Configurable // Panels
public class blue12ball extends OpMode {
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
        follower.setStartingPose(new Pose(39, 153, Math.toRadians(180)));

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
        public PathChain pickup11;
        public PathChain pickup12;
        public PathChain shoot2;
        public PathChain gate;
        public PathChain pickup21;
        public PathChain pickup22;
        public PathChain shoot3;
        public PathChain pickup3;
        public PathChain shoot4;

        public Paths(Follower follower) {
            shoot1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(39.000, 153.000),

                                    new Pose(60.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))

                    .build();

            pickup11 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(60.000, 84.000),

                                    new Pose(60.000, 59.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))

                    .build();

            pickup12 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(60.000, 59.000),

                                    new Pose(10.000, 59.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            shoot2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(10.000, 59.000),

                                    new Pose(64.000, 80.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))

                    .build();

            gate = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(64.000, 80.000),

                                    new Pose(10.000, 64.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))

                    .build();

            pickup21 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(10.000, 69.000),

                                    new Pose(52.000, 35.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            pickup22 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(52.000, 35.000),

                                    new Pose(5.000, 35.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            shoot3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(5.000, 35.000),

                                    new Pose(60.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))

                    .build();

            pickup3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(60.000, 84.000),

                                    new Pose(12.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))

                    .build();

            shoot4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(12.000, 84.000),

                                    new Pose(60.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))

                    .build();
        }
    }






    public void autonomousPathUpdate() {
        switch (pathState){
            case 0:
                if (pathTimer.getElapsedTime() > 1000) {
                    follower.followPath(paths.shoot1, 0.7, false);
                    setPathState(2);
                }
                break; // IMPORTANT!

            case 2:
                if(!follower.isBusy()) {
                    setPathState(3);
                }
                break; // IMPORTANT!
            case 3:
                if (pathTimer.getElapsedTime() > 2000){

                    flyWheel.setPower(0.0);
                    follower.followPath(paths.pickup11, 0.7, true);
                    setPathState(4);
                }
                else{
                    flyWheel.setPower(1.0);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.pickup12);
                    setPathState(5);
                }
                break; // IMPORTANT!
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.shoot2, 0.7, true);
                    setPathState(6);
                }
                break; // IMPORTANT!
            case 6:
                if(!follower.isBusy()) {
                    setPathState(7);
                }
                break; // IMPORTANT!
            case 7:
                if (pathTimer.getElapsedTime() > 2000){

                    flyWheel.setPower(0.0);
                    follower.followPath(paths.gate, 0.7, true);
                    setPathState(8);
                }
                else{
                    flyWheel.setPower(1.0);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(paths.pickup21, 0.7, true);
                    setPathState(9);
                }
                break; // IMPORTANT!
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(paths.pickup22, 0.7, true);
                    setPathState(10);
                }
                break; // IMPORTANT!
            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(paths.shoot3, 0.7, true);
                    setPathState(11);
                }
                break; // IMPORTANT!
            case 11:
                if(!follower.isBusy()) {
                    setPathState(12);
                }
                break; // IMPORTANT!
            case 12:
                if (pathTimer.getElapsedTime() > 2000){

                    flyWheel.setPower(0.0);
                    follower.followPath(paths.pickup3, 0.7, true);
                    setPathState(13);
                }
                else{
                    flyWheel.setPower(1.0);
                }
                break;
                case 13:
                if (!follower.isBusy()) {
                    follower.followPath(paths.shoot4, 0.7, true);
                    setPathState(14);
                }
                break; // IMPORTANT!
            case 14:
                if(!follower.isBusy()) {
                    setPathState(15);
                }
                break; // IMPORTANT!
            case 15:
                if (pathTimer.getElapsedTime() > 2000){

                    flyWheel.setPower(0.0);
                    follower.followPath(paths.pickup3, 0.7, true);
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
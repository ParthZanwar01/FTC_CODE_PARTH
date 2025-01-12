package pedroPathing.actual_code;

import static java.lang.Thread.sleep;

import android.view.Gravity;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Config
@Autonomous(name = "AutoRight")
public class AutoRight extends OpMode {

    private int step = 0;

    private Follower follower;

    private Servo ScoringClaw, ScoringWrist, ScoringArm, SubClaw, SubWrist, SubAngle;

    private Path clipOnFirstClip, PathToBlock1Part1, PathToBlock1Part2, PathToBlock1Part3, PushFirstBlockBack, PathToBlock2Part1, PathToBlock2Part2, PushSecondBlockBack, GrabSecondClip, ClipOnSecondClipPart1, ClipOnSecondClipPart2, GrabThirdClip, ClipOnThirdClip, GrabFourthClip , ClipOnFourthClip, ParkRobot;
    private Path gamePreload;

    ElapsedTime timer = new ElapsedTime();
    boolean timerReset;

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        ScoringWrist = hardwareMap.get(Servo.class, "ScoringWrist");
        ScoringArm = hardwareMap.get(Servo.class, "ScoringArm");
        ScoringClaw = hardwareMap.get(Servo.class, "ScoringClaw");
        SubAngle = hardwareMap.get(Servo.class, "SubAngle");
        SubWrist = hardwareMap.get(Servo.class, "SubWrist");
        SubClaw = hardwareMap.get(Servo.class, "SubClaw");


        gamePreload = new Path(new BezierLine(new Point(0, 0, Point.CARTESIAN), new Point(-15, -6, Point.CARTESIAN)));
        gamePreload.setConstantHeadingInterpolation(0);
        clipOnFirstClip = new Path(new BezierLine(new Point(-15,-6, Point.CARTESIAN), new Point(-33,-12.34, Point.CARTESIAN)));
        clipOnFirstClip.setConstantHeadingInterpolation(0);
        PathToBlock1Part1 = new Path(new BezierLine(new Point(-33, -12.34, Point.CARTESIAN), new Point(-3, 21.13, Point.CARTESIAN)));
        PathToBlock1Part1.setConstantHeadingInterpolation(0);
        PathToBlock1Part2 = new Path(new BezierLine(new Point(-3, 21.13, Point.CARTESIAN), new Point(-51.98, 21.13, Point.CARTESIAN)));
        PathToBlock1Part2.setConstantHeadingInterpolation(0);
        PathToBlock1Part3 = new Path(new BezierLine(new Point(-51.98, 21.13, Point.CARTESIAN), new Point(-51.98, 30.00, Point.CARTESIAN)));
        PathToBlock1Part3.setConstantHeadingInterpolation(0);
        PushFirstBlockBack = new Path(new BezierLine(new Point(-51.98, 30.00, Point.CARTESIAN), new Point(-3.6, 30.00, Point.CARTESIAN)));
        PushFirstBlockBack.setConstantHeadingInterpolation(0);
        PathToBlock2Part1 = new Path(new BezierLine(new Point(-3.6, 30.0, Point.CARTESIAN), new Point(-51.98, 28.0, Point.CARTESIAN)));
        PathToBlock2Part1.setConstantHeadingInterpolation(0);
        PathToBlock2Part2 = new Path(new BezierLine(new Point(-51.98, 30.0, Point.CARTESIAN), new Point(-51.98, 41.00)));
        PathToBlock2Part2.setConstantHeadingInterpolation(0);
        PushSecondBlockBack = new Path(new BezierLine(new Point(-51.98, 41.00, Point.CARTESIAN), new Point(-8.49, 41.00, Point.CARTESIAN)));
        PushSecondBlockBack.setConstantHeadingInterpolation(0);
        GrabSecondClip = new Path(new BezierLine(new Point(-8.49, 41.00, Point.CARTESIAN), new Point(/*-3.7*/ -8.49, 25.42, Point.CARTESIAN)));
        GrabSecondClip.setConstantHeadingInterpolation(0);
        ClipOnSecondClipPart1 = new Path(new BezierLine(new Point(/*-3.7*/-8.49, 25.42, Point.CARTESIAN), new Point(-15, -6, Point.CARTESIAN)));
        ClipOnSecondClipPart1.setConstantHeadingInterpolation(0);
        ClipOnSecondClipPart2 = new Path(new BezierLine(new Point(-15, -6, Point.CARTESIAN), new Point(-33, -14, Point.CARTESIAN)));
        ClipOnSecondClipPart2.setConstantHeadingInterpolation(0);
        GrabThirdClip = new Path(new BezierLine(new Point(-33, -14, Point.CARTESIAN), new Point(-3.25, 25.42, Point.CARTESIAN)));
        GrabThirdClip.setConstantHeadingInterpolation(0);
        ClipOnThirdClip = new Path(new BezierLine(new Point(-3.25, 25.42, Point.CARTESIAN), new Point(-33, -15, Point.CARTESIAN)));
        ClipOnThirdClip.setConstantHeadingInterpolation(0);
        GrabFourthClip = new Path(new BezierLine(new Point(-33, -15, Point.CARTESIAN), new Point(-3.75, 25.42, Point.CARTESIAN)));
        GrabFourthClip.setConstantHeadingInterpolation(0);
        ClipOnFourthClip = new Path(new BezierLine(new Point(-3.75, 25.42, Point.CARTESIAN), new Point(-33, -16, Point.CARTESIAN)));
        ClipOnFourthClip.setConstantHeadingInterpolation(0);
        ParkRobot = new Path(new BezierLine(new Point(-32.33, -16, Point.CARTESIAN), new Point(-3.75, 41.00, Point.CARTESIAN)));
        ParkRobot.setConstantHeadingInterpolation(0);
        /*
        backwards = new Path(new BezierLine(new Point(DISTANCE,0, Point.CARTESIAN), new Point(0,0, Point.CARTESIAN)));
        backwards.setConstantHeadingInterpolation(0);
        */

    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
    }

    public void autonomousPathUpdate(){
        switch (step) {
            case (0):
                follower.followPath(gamePreload);
                setStep(1);
                timerReset = true;
            case (1):
                if (timerReset){
                    timer.reset();
                    timerReset = false;
                }
                if (timer.milliseconds() > 0) {
                    ScoringClaw.setPosition(0.55);
                    ScoringWrist.setPosition(0.58);
                    ScoringArm.setPosition(0.9);
                }
                if (timer.milliseconds() > 2000) {
                    follower.followPath(clipOnFirstClip, true);
                    setStep(2);
                    timerReset = true;
                }
                break;
            case (2):
                if (timerReset){
                    timer.reset();
                    timerReset = false;
                }
                if (timer.milliseconds() > 500) ScoringClaw.setPosition(0.8);
                if (timer.milliseconds() > 1000){
                    follower.followPath(PathToBlock1Part1, true);
                    setStep(3);
                    timerReset = true;
                }
                break;
            case (3):
                if (timerReset){
                    timer.reset();
                    timerReset = false;
                }
                if (timer.milliseconds() > 1000) {
                    follower.followPath(PathToBlock1Part2, true);
                    timerReset = true;
                    setStep(4);
                }
                break;
            case (4):
                if (timerReset){
                    timer.reset();
                    timerReset = false;
                }
                if (timer.milliseconds() > 1000) {
                    follower.followPath(PathToBlock1Part3, true);
                    timerReset = true;
                    setStep(5);
                }
                break;
            case (5):
                if (timerReset){
                    timer.reset();
                    timerReset = false;
                }
                if (timer.milliseconds() > 1000) {
                    follower.followPath(PushFirstBlockBack, true);
                    timerReset = true;
                    setStep(6);
                }
                break;
            case(6):
                if (timerReset){
                    timer.reset();
                    timerReset = false;
                }
                if (timer.milliseconds() > 1000){
                    follower.followPath(PathToBlock2Part1, true);
                    timerReset = true;
                    setStep(7);
                }
                break;
            case(7):
                if (timerReset){
                    timer.reset();
                    timerReset = false;
                }
                if (timer.milliseconds() > 1000){
                    follower.followPath(PathToBlock2Part2, true);
                    timerReset = true;
                    setStep(8);
                }
                break;
            case (8):
                if (timerReset){
                    timer.reset();
                    timerReset = false;
                }
                if (timer.milliseconds() > 1000){ // Replace with correct values
                    follower.followPath(PushSecondBlockBack, true);
                    timerReset = true;
                    setStep(9);
                }
                break;
            case (9):
                if (timerReset){
                    timer.reset();
                    timerReset = false;
                }
                if (timer.milliseconds() > 1000){
                    follower.followPath(GrabSecondClip, true);
                    timerReset = true;
                    setStep(10);
                }
                break;
            case (10):
                if (timerReset){
                    timer.reset();
                    timerReset = false;
                }
                if (timer.milliseconds() > 0 && timer.milliseconds() < 2000){
                    ScoringWrist.setPosition(0.0);
                }
                if (timer.milliseconds() > 1000){
                    ScoringArm.setPosition(0.1);
                }
                if (timer.milliseconds() > 2000){
                    ScoringWrist.setPosition(0.95);
                }
                if (timer.milliseconds() > 4000) {
                    ScoringClaw.setPosition(0.55);
                }
                if (timer.milliseconds() > 5000) {
                    ScoringWrist.setPosition(0.56);
                    ScoringArm.setPosition(0.9);
                }
                if (timer.milliseconds() > 6000){
                    follower.followPath(ClipOnSecondClipPart1, true);
                    timerReset = true;
                    setStep(11);
                }
                break;
            case (11):
                if (timerReset){
                    timer.reset();
                    timerReset = false;
                }

                if (timer.milliseconds() > 0){
                    ScoringClaw.setPosition(0.6);
                    ScoringWrist.setPosition(0.57);
                    ScoringArm.setPosition(0.9);
                }
                if (timer.milliseconds() > 2000){
                    ScoringClaw.setPosition(0.55);
                }
                if (timer.milliseconds() > 3000){
                    follower.followPath(ClipOnSecondClipPart2, true);
                    timerReset = true;
                    setStep(12);
                }
                break;
            case (12):
                if (timerReset){
                    timer.reset();
                    timerReset = false;
                }

                if (timer.milliseconds() > 1000) {
                    ScoringClaw.setPosition(0.8);
                }
                if (timer.milliseconds() > 2000){
                    ScoringArm.setPosition(0.1);
                    follower.followPath(GrabThirdClip, true);
                    timerReset = true;
                    setStep(13);
                }
                break;
            case (13):
                if (timerReset){
                    timer.reset();
                    timerReset = false;
                }
                if (timer.milliseconds() > 1000 && timer.milliseconds() < 2000){
                    ScoringWrist.setPosition(0.0);
                }
                 if (timer.milliseconds() > 2000){
                     ScoringWrist.setPosition(0.95);
                 }

                 if (timer.milliseconds() > 2500){
                     ScoringClaw.setPosition(0.4);
                 }

                if (timer.milliseconds() > 3000) {
                    ScoringWrist.setPosition(0.58);
                    ScoringArm.setPosition(0.9);
                    follower.followPath(ClipOnThirdClip, true);
                    timerReset = true;
                    setStep(-1);
                }
                break;
            case (14):
                break;
        }
    }

    public void setStep(int step){
        this.step = step;
    }
}

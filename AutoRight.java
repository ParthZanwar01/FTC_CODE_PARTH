package pedroPathing.actual_code;

import com.acmerobotics.dashboard.config.Config;
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

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Config
@Autonomous(name = "AutoRight")
public class AutoRight extends OpMode {

    private int step = 0;

    private Follower follower;

    private Servo ScoringClaw, ScoringWrist, ScoringArm, SubClaw, SubWrist, SubAngle;

    private Path clipOnFirstClip, PushFirstBlockBack, GrabThirdClip;
    private Path gamePreload;
    private Path CurveToFirstBlockPart1, CurveToFirstBlockPart2, CurveToSecondBlock, CurveToPushAndGrabSecond, CurveToThirdBlock, CurveToPushBackThird, CurveToHookSecond, CurveToHookThird, CurveToGrabFourth, CurveToHookFourth, CurveToGrabFifth, CurveToHookFifth;

    ElapsedTime timer = new ElapsedTime();
    boolean timerReset;
    
    private final double ScoringServoPosition = 0.98;

    private final double GrabbingServoPosition = 0.03;

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

        gamePreload = new Path(new BezierLine(new Point(0, 0, Point.CARTESIAN), new Point(15, 3, Point.CARTESIAN)));
        gamePreload.setConstantHeadingInterpolation(0);
        clipOnFirstClip = new Path(new BezierLine(new Point(15,3, Point.CARTESIAN), new Point(31,20, Point.CARTESIAN)));
        clipOnFirstClip.setConstantHeadingInterpolation(0);
        CurveToFirstBlockPart1 = new Path(new BezierCurve(new Point(31, 20, Point.CARTESIAN), new Point(20,20, Point.CARTESIAN), new Point(3, -21.13, Point.CARTESIAN)));
        CurveToFirstBlockPart1.setConstantHeadingInterpolation(0);
        CurveToFirstBlockPart2 = new Path(new BezierCurve(new Point(3, -21.13, Point.CARTESIAN), new Point(55, -21.13, Point.CARTESIAN), new Point(48, -30.00, Point.CARTESIAN)));
        CurveToFirstBlockPart2.setConstantHeadingInterpolation(0);
        PushFirstBlockBack = new Path(new BezierLine(new Point(48, -30.00, Point.CARTESIAN), new Point(6, -30.00, Point.CARTESIAN)));
        PushFirstBlockBack.setConstantHeadingInterpolation(0);
        CurveToSecondBlock = new Path(new BezierCurve(new Point(6, -30.0, Point.CARTESIAN), new Point(50, -32, Point.CARTESIAN), new Point(48, -41.00, Point.CARTESIAN)));
        CurveToSecondBlock.setConstantHeadingInterpolation(0);
        CurveToPushAndGrabSecond = new Path(new BezierCurve(new Point(48, -41.0, Point.CARTESIAN), new Point(6.7, -41.0, Point.CARTESIAN)));
        CurveToPushAndGrabSecond.setConstantHeadingInterpolation(0);
        CurveToThirdBlock = new Path(new BezierCurve(new Point(6.7, -41.0, Point.CARTESIAN), new Point(48, -43.0, Point.CARTESIAN), new Point(48, -46, Point.CARTESIAN)));
        CurveToThirdBlock.setConstantHeadingInterpolation(0);
        CurveToPushBackThird = new Path(new BezierCurve(new Point(48, -46, Point.CARTESIAN), new Point(2.5, -46, Point.CARTESIAN)));
        CurveToPushBackThird.setConstantHeadingInterpolation(0);
        CurveToHookSecond = new Path(new BezierCurve(new Point(2.5, -46, Point.CARTESIAN), new Point(15, 10, Point.CARTESIAN), new Point(29, 19, Point.CARTESIAN)));
        CurveToHookSecond.setConstantHeadingInterpolation(0);
        GrabThirdClip = new Path(new BezierCurve(new Point(29, 19, Point.CARTESIAN), new Point( 20, 19, Point.CARTESIAN), new Point(15,6, Point.CARTESIAN), new Point(3, -18.00, Point.CARTESIAN)));
        GrabThirdClip.setConstantHeadingInterpolation(0);
        CurveToHookThird = new Path(new BezierCurve(new Point(3, -18.00, Point.CARTESIAN), new Point(10,6,Point.CARTESIAN),new Point(15, 5, Point.CARTESIAN), new Point(32.5, 16, Point.CARTESIAN)));
        CurveToHookThird.setConstantHeadingInterpolation(0);
        CurveToGrabFourth = new Path(new BezierCurve(new Point(32.5, 14, Point.CARTESIAN), new Point(15, 0, Point.CARTESIAN), new Point(5,-10, Point.CARTESIAN), new Point(3.75, -18.0, Point.CARTESIAN)));
        CurveToGrabFourth.setConstantHeadingInterpolation(0);
        CurveToHookFourth = new Path(new BezierCurve(new Point(3.75, -18.0, Point.CARTESIAN), new Point(10,3, Point.CARTESIAN), new Point(15, 6, Point.CARTESIAN), new Point(32.5, 14, Point.CARTESIAN)));
        CurveToHookFourth.setConstantHeadingInterpolation(0);
        CurveToGrabFifth = new Path(new BezierCurve(new Point(32, 13, Point.CARTESIAN), new Point(10, 6, Point.CARTESIAN), new Point(7.1,-10, Point.CARTESIAN), new Point(4, -18.0, Point.CARTESIAN)));
        CurveToGrabFifth.setConstantHeadingInterpolation(0);
        CurveToHookFifth = new Path(new BezierCurve(new Point(4, -18.0, Point.CARTESIAN),new Point(10,3 , Point.CARTESIAN), new Point(15, 6, Point.CARTESIAN), new Point(33, 12, Point.CARTESIAN)));
        CurveToHookFifth.setConstantHeadingInterpolation(0);
        follower.setMaxPower(1.0);
        timerReset = true;

    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        follower.update();
        telemetry.addData("Pose: ", follower.getPose());
        autonomousPathUpdate();
    }

    public void autonomousPathUpdate(){
        switch (step) {
            case (0): {
                if (timerReset) {
                    timer.reset();
                    timerReset = false;
                }
                if (timer.milliseconds() > 0) {
                    ScoringClaw.setPosition(0.55);
                    ScoringWrist.setPosition(ScoringServoPosition);
                    ScoringArm.setPosition(0.95);
                }
                if (timer.milliseconds() > 500){
                    follower.followPath(gamePreload, true);
                    setStep(1);
                    timerReset = true;
                }
                break;
            }
            case (1): {
                if (timerReset) {
                    timer.reset();
                    timerReset = false;
                }

                if (gamePreload.isAtParametricEnd()) {
                    follower.followPath(clipOnFirstClip, true);
                    setStep(2);
                    timerReset = true;
                }
                break;
            }
            case (2): {
                if (timerReset) {
                    timer.reset();
                    timerReset = false;
                }
                if (clipOnFirstClip.isAtParametricEnd()) {
                    ScoringClaw.setPosition(0.8);
                    follower.followPath(CurveToFirstBlockPart1, true);
                    setStep(3);
                    timerReset = true;
                }
                break;
            }
            case (3): {
                if (timerReset) {
                    timer.reset();
                    timerReset = false;
                }
                if (CurveToFirstBlockPart1.isAtParametricEnd()) {
                    follower.followPath(CurveToFirstBlockPart2, true);
                    setStep(4);
                    timerReset = true;
                }
                break;
            }
            case (4): {
                if (timerReset) {
                    timer.reset();
                    timerReset = false;
                }
                if (CurveToFirstBlockPart2.isAtParametricEnd()) {
                    follower.followPath(PushFirstBlockBack, true);
                    setStep(5);
                    timerReset = true;
                }
                break;
            }
            case (5): {
                if (timerReset) {
                    timer.reset();
                    timerReset = false;
                }
                if (PushFirstBlockBack.isAtParametricEnd()) {
                    follower.followPath(CurveToSecondBlock, true);
                    timerReset = true;
                    setStep(6);
                }
                break;
            }
            case (6): {
                if (timerReset) {
                    timer.reset();
                    timerReset = false;
                }

                if (CurveToSecondBlock.isAtParametricEnd()) {
                    follower.followPath(CurveToPushAndGrabSecond, true);
                    timerReset = true;
                    setStep(7);
                }
                break;
            }
            case (7): {
                if (timerReset) {
                    timer.reset();
                    timerReset = false;
                }

                if (CurveToPushAndGrabSecond.isAtParametricEnd()) {
                    ScoringArm.setPosition(0.15);
                    follower.followPath(CurveToThirdBlock, true);
                    timerReset = true;
                    setStep(8);
                }
                break;
            }
            case (8):{
                if (timerReset) {
                    timer.reset();
                    timerReset = false;
                }

                if (CurveToThirdBlock.isAtParametricEnd()) {
                    ScoringWrist.setPosition(ScoringServoPosition);

                    follower.followPath(CurveToPushBackThird, true);
                    timerReset = true;
                    setStep(9);
                }
                break;
            }
            case (9): {
                if (timerReset) {
                    timer.reset();
                    timerReset = false;
                }

                if (timer.milliseconds() > 0) {
                    ScoringWrist.setPosition(GrabbingServoPosition);
                }

                if (timer.milliseconds() > 1600){
                    ScoringClaw.setPosition(0.55);
                }
                if (timer.milliseconds() > 2400) {
                    ScoringWrist.setPosition(ScoringServoPosition);
                    ScoringArm.setPosition(0.95);
                    follower.followPath(CurveToHookSecond, true);
                    timerReset = true;
                    setStep(10);
                }
                break;
            }
            case (10): {
                if (timerReset) {
                    timer.reset();
                    timerReset = false;
                }
                if (CurveToHookSecond.isAtParametricEnd()) {
                    ScoringClaw.setPosition(0.8);
                    follower.followPath(GrabThirdClip, true);
                    timerReset = true;
                    setStep(11);
                }
                break;
            }
            case (11): {
                if (timerReset) {
                    timer.reset();
                    timerReset = false;
                }
                if (timer.milliseconds() > 0 && timer.milliseconds() < 2000) {
                    ScoringWrist.setPosition(ScoringServoPosition);
                }
                if (timer.milliseconds() > 1000 && timer.milliseconds() < 5000) {
                    ScoringArm.setPosition(0.15);
                }
                if (timer.milliseconds() > 2000 && timer.milliseconds() < 3000) {
                    ScoringWrist.setPosition(GrabbingServoPosition);
                }
                if (timer.milliseconds() > 2500) {
                    ScoringClaw.setPosition(0.50);
                }
                if (timer.milliseconds() > 3000) {
                    ScoringWrist.setPosition(ScoringServoPosition);
                    ScoringArm.setPosition(0.95);
                }
                if (timer.milliseconds() > 3500) {
                    follower.followPath(CurveToHookThird, true);
                    timerReset = true;
                    setStep(12);
                }
                break;
            }
            case (12): {
                if (timerReset) {
                    timer.reset();
                    timerReset = false;
                }

                if (CurveToHookThird.isAtParametricEnd()) {
                    ScoringClaw.setPosition(0.8);
                    telemetry.update();
                    follower.followPath(CurveToGrabFourth, true);
                    timerReset = true;
                    setStep(13);
                }
                break;
            }
            case (13): {
                if (timerReset) {
                    timer.reset();
                    timerReset = false;
                }

                if (timer.milliseconds() > 0 && timer.milliseconds() < 2000) {
                    ScoringWrist.setPosition(ScoringServoPosition);
                }
                if (timer.milliseconds() > 1000 && timer.milliseconds() < 5000) {
                    ScoringArm.setPosition(0.15);
                }
                if (timer.milliseconds() > 2000 && timer.milliseconds() < 3000) {
                    ScoringWrist.setPosition(GrabbingServoPosition);
                }
                if (timer.milliseconds() > 2500) {
                    ScoringClaw.setPosition(0.55);
                }
                if (timer.milliseconds() > 3000) {
                    ScoringWrist.setPosition(ScoringServoPosition);
                    ScoringArm.setPosition(0.95);
                }

                if (timer.milliseconds() > 3500) {
                    follower.followPath(CurveToHookFourth, true);
                    setStep(14);
                    timerReset = true;
                }
                break;
            }
            case (14): {
                if (timerReset) {
                    timer.reset();
                    timerReset = false;
                }

                if (CurveToHookFourth.isAtParametricEnd()) {
                    ScoringClaw.setPosition(0.8);
                    follower.followPath(CurveToGrabFifth, true);
                    timerReset = true;
                    setStep(15);
                }
                break;
            }
            case (15): {
                if (timerReset) {
                    timer.reset();
                    timerReset = false;
                }

                if (timer.milliseconds() > 0 && timer.milliseconds() < 2000) {
                    ScoringWrist.setPosition(ScoringServoPosition);
                }
                if (timer.milliseconds() > 1000 && timer.milliseconds() < 5000) {
                    ScoringArm.setPosition(0.15);
                }
                if (timer.milliseconds() > 2000 && timer.milliseconds() < 3000) {
                    ScoringWrist.setPosition(GrabbingServoPosition);
                }
                if (timer.milliseconds() > 2500) {
                    ScoringClaw.setPosition(0.55);
                }
                if (timer.milliseconds() > 3000) {
                    ScoringWrist.setPosition(ScoringServoPosition);
                    ScoringArm.setPosition(0.95);
                }

                if (timer.milliseconds() > 3500) {
                    follower.followPath(CurveToHookFifth, true);
                    setStep(16);
                    timerReset = true;
                }
                break;
            }
            case (16): {
                if (CurveToHookFifth.isAtParametricEnd()){
                    ScoringClaw.setPosition(0.8);
                }
                break;
            }
        }
    }

    public void setStep(int step){
        this.step = step;
    }
}

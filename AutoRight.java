package pedroPathing.actual_code;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
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

    private Path clipOnFirstClip, PushFirstBlockBack, GrabThirdClip, ParkRobot;
    private Path gamePreload;
    private Path CurveToFirstBlockPart1, CurveToFirstBlockPart2, CurveToSecondBlock, CurveToPushAndGrabSecond, CurveToHookSecond, CurveToHookThird, CurveToGrabFourth, CurveToHookFourth;

    ElapsedTime timer = new ElapsedTime();
    boolean timerReset;
    
    private double ScoringServoPosition = 0.96;

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


        gamePreload = new Path(new BezierLine(new Point(0, 0, Point.CARTESIAN), new Point(-15, -3, Point.CARTESIAN)));
        gamePreload.setConstantHeadingInterpolation(0);
        clipOnFirstClip = new Path(new BezierLine(new Point(-15,-3, Point.CARTESIAN), new Point(-31.5,-17, Point.CARTESIAN)));
        clipOnFirstClip.setConstantHeadingInterpolation(0);
        CurveToFirstBlockPart1 = new Path(new BezierCurve(new Point(-31.5, -17, Point.CARTESIAN), new Point(-3, 21.13, Point.CARTESIAN)));
        CurveToFirstBlockPart1.setConstantHeadingInterpolation(0);
        CurveToFirstBlockPart2 = new Path(new BezierCurve(new Point(-3, 21.13, Point.CARTESIAN), new Point(-55, 21.13, Point.CARTESIAN), new Point(-55, 30.00, Point.CARTESIAN)));
        CurveToFirstBlockPart2.setConstantHeadingInterpolation(0);
        PushFirstBlockBack = new Path(new BezierLine(new Point(-55, 30.00, Point.CARTESIAN), new Point(-5, 30.00, Point.CARTESIAN)));
        PushFirstBlockBack.setConstantHeadingInterpolation(0);
        CurveToSecondBlock = new Path(new BezierCurve(new Point(-5, 30.0, Point.CARTESIAN), new Point(-55, 28.0, Point.CARTESIAN), new Point(-55, 37.00, Point.CARTESIAN)));
        CurveToSecondBlock.setConstantHeadingInterpolation(0);
        CurveToPushAndGrabSecond = new Path(new BezierCurve(new Point(-52, 41.0, Point.CARTESIAN), new Point(-4.2, 41.0, Point.CARTESIAN)));
        CurveToPushAndGrabSecond.setConstantHeadingInterpolation(0);
        CurveToHookSecond = new Path(new BezierCurve(new Point(-5, 25.42, Point.CARTESIAN), new Point(-15, -6, Point.CARTESIAN), new Point(-30, -17, Point.CARTESIAN)));
        CurveToHookSecond.setConstantHeadingInterpolation(0);
        GrabThirdClip = new Path(new BezierCurve(new Point(-30, -17, Point.CARTESIAN), new Point(-15,0, Point.CARTESIAN), new Point(-4.9, 20.00, Point.CARTESIAN)));
        GrabThirdClip.setConstantHeadingInterpolation(0);
        CurveToHookThird = new Path(new BezierCurve(new Point(-5, 20.00, Point.CARTESIAN), new Point(-10,0,Point.CARTESIAN),new Point(-15, -6, Point.CARTESIAN), new Point(-31, -15, Point.CARTESIAN)));
        CurveToHookThird.setConstantHeadingInterpolation(0);
        CurveToGrabFourth = new Path(new BezierCurve(new Point(-30, -9, Point.CARTESIAN), new Point(-10, 0, Point.CARTESIAN), new Point(-7,10, Point.CARTESIAN), new Point(-4.9, 20.0, Point.CARTESIAN)));
        CurveToGrabFourth.setConstantHeadingInterpolation(0);
        CurveToHookFourth = new Path(new BezierCurve(new Point(-5.5, 20.0, Point.CARTESIAN),new Point(-10,7 , Point.CARTESIAN), new Point(-15, -6, Point.CARTESIAN), new Point(-31, -12, Point.CARTESIAN)));
        CurveToHookFourth.setConstantHeadingInterpolation(0);
        ParkRobot = new Path(new BezierLine(new Point(-31, -12, Point.CARTESIAN), new Point(-3.75, 41.0, Point.CARTESIAN)));
        ParkRobot.setConstantHeadingInterpolation(0);
        follower.setMaxPower(1.0);

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
            case (0): {
                follower.followPath(gamePreload, true);
                setStep(1);
                timerReset = true;
                break;
            }
            case (1): {
                if (timerReset) {
                    timer.reset();
                    timerReset = false;
                }
                if (timer.milliseconds() > 0) {
                    ScoringClaw.setPosition(0.55);
                    ScoringWrist.setPosition(ScoringServoPosition);
                    ScoringArm.setPosition(0.95);
                }
                if (timer.milliseconds() > 1750) {
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
                if (timer.milliseconds() > 750) {
                    ScoringClaw.setPosition(0.8);
                }
                if (timer.milliseconds() > 1250) {
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
                if (timer.milliseconds() > 1000) {
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
                if (timer.milliseconds() > 1500) {
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
                if (timer.milliseconds() > 1250) {
                    follower.followPath(CurveToSecondBlock, true);
                    timerReset = true;
                    setStep(6);
                    follower.setMaxPower(0.7);
                }
                break;
            }
            case (6): {
                if (timerReset) {
                    timer.reset();
                    timerReset = false;
                }

                if (timer.milliseconds() > 0 && timer.milliseconds() < 1000) {
                    ScoringWrist.setPosition(ScoringServoPosition);
                    ScoringArm.setPosition(0.15);
                }

                if (timer.milliseconds() > 2000) {
                    follower.followPath(CurveToPushAndGrabSecond, true);
                    timerReset = true;
                    setStep(7);
                    follower.setMaxPower(1.0);
                }
                break;
            }
            case (7): {
                if (timerReset) {
                    timer.reset();
                    timerReset = false;
                }

                if (timer.milliseconds() > 1500) {
                    ScoringWrist.setPosition(0.13);
                }
                if (timer.milliseconds() > 2000) {
                    ScoringClaw.setPosition(0.55);
                }
                if (timer.milliseconds() > 2500) {
                    ScoringWrist.setPosition(ScoringServoPosition);
                    ScoringArm.setPosition(0.95);
                }
                if (timer.milliseconds() > 3250) {
                    follower.followPath(CurveToHookSecond, true);
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

                if (timer.milliseconds() > 2500) {
                    ScoringClaw.setPosition(0.8);
                }
                if (timer.milliseconds() > 2800) {
                    ScoringArm.setPosition(0.15);
                    follower.followPath(GrabThirdClip, true);
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
                if (timer.milliseconds() > 0 && timer.milliseconds() < 2000) {
                    ScoringWrist.setPosition(ScoringServoPosition);
                }
                if (timer.milliseconds() > 1000 && timer.milliseconds() < 5000) {
                    ScoringArm.setPosition(0.15);
                }
                if (timer.milliseconds() > 2000 && timer.milliseconds() < 5000) {
                    ScoringWrist.setPosition(0.13);
                }
                if (timer.milliseconds() > 3000) {
                    ScoringClaw.setPosition(0.50);
                }
                if (timer.milliseconds() > 4000) {
                    ScoringWrist.setPosition(ScoringServoPosition);
                    ScoringArm.setPosition(0.95);
                }
                if (timer.milliseconds() > 4500) {
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

                if (timer.milliseconds() > 2250) {
                    ScoringClaw.setPosition(0.8);
                }

                if (timer.milliseconds() > 3000) {
                    ScoringArm.setPosition(0.15);
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
                if (timer.milliseconds() > 2000 && timer.milliseconds() < 5000) {
                    ScoringWrist.setPosition(0.12);
                }
                if (timer.milliseconds() > 3000) {
                    ScoringClaw.setPosition(0.55);
                }
                if (timer.milliseconds() > 4000) {
                    ScoringWrist.setPosition(ScoringServoPosition);
                    ScoringArm.setPosition(0.95);
                }

                if (timer.milliseconds() > 4500) {
                    follower.followPath(CurveToHookFourth, true);
                    setStep(14);
                    timerReset = true;
                }
                break;
            }
            case (14):
                if (timerReset){
                    timer.reset();
                    timerReset = false;
                }

                if (timer.milliseconds() > 2000){
                    ScoringClaw.setPosition(0.8);
                }

        }
    }

    public void setStep(int step){
        this.step = step;
    }
}

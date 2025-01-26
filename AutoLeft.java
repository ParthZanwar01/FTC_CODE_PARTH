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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Config
@Autonomous(name = "AutoLeft")
public class AutoLeft extends OpMode {


    static final double COUNTS_PER_MOTOR_REV_ScoringSlides = 537.6;
    static final double INCHES_PER_REV_ScoringSlides = 3.6;        // Slide moves 1 inch per revolution
    static final double COUNTS_PER_INCH_ScoringSlides = COUNTS_PER_MOTOR_REV_ScoringSlides / INCHES_PER_REV_ScoringSlides;
    static final double COUNTS_PER_MOTOR_REV_SubSlides = 384.5;
    static final double INCHES_PER_REV_SubSlides = 5.15;
    static final double COUNTS_PER_INCH_SubSlides = COUNTS_PER_MOTOR_REV_SubSlides / INCHES_PER_REV_SubSlides;
    static int TARGET_POSITION_SCORING_SLIDES = 0;
    static double TARGET_POSITION_SUB_SLIDES = 0;
    ElapsedTime timer = new ElapsedTime();
    boolean timerReset;
    boolean ScoringSlideReset;
    boolean SubSlideReset;
    private int step = 0;
    private Follower follower;
    private Servo ScoringClaw, ScoringWrist, ScoringArm, SubClaw, SubWrist, SubAngle;
    private DcMotor ScoringSlidesMotor, SubSlides;
    private Point beforeScore, Score;
    private Path scorePreload;
    private Path gamePreload;
    private Path CurveToFirstBlock, CurveToScoreFirst, CurveToSecondBlock, CurveToScoreSecond, CurveToThirdBlock, CurveToScoreThird;

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
        ScoringSlidesMotor = hardwareMap.get(DcMotor.class, "ScoringSlides");
        SubSlides = hardwareMap.get(DcMotor.class, "SubSlides");

        //Set Slides Directions
        SubSlides.setDirection(DcMotor.Direction.REVERSE);
        ScoringSlidesMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //Set Slide Modes
        ScoringSlidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ScoringSlidesMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ScoringSlidesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        beforeScore = new Point(0, -0.1, Point.CARTESIAN);
        Score = new Point(0, -5, Point.CARTESIAN);


        gamePreload = new Path(new BezierLine(new Point(0, 0, Point.CARTESIAN), beforeScore));
        gamePreload.setConstantHeadingInterpolation(0);

        scorePreload = new Path(new BezierLine(beforeScore, Score));
        scorePreload.setConstantHeadingInterpolation(0);

        CurveToFirstBlock = new Path(new BezierCurve(Score, beforeScore, new Point(-11.7647, -1, Point.CARTESIAN)));
        CurveToFirstBlock.setConstantHeadingInterpolation(0);

        CurveToScoreFirst = new Path(new BezierCurve(new Point(-11.7647, -1, Point.CARTESIAN), beforeScore, Score));
        CurveToScoreFirst.setConstantHeadingInterpolation(0);

        CurveToSecondBlock = new Path(new BezierCurve(Score, beforeScore, new Point(-11.7647, -2, Point.CARTESIAN)));
        CurveToSecondBlock.setConstantHeadingInterpolation(0);

        CurveToScoreSecond = new Path(new BezierCurve(new Point(-11.7647, -17.0862, Point.CARTESIAN), beforeScore, Score));
        CurveToScoreSecond.setConstantHeadingInterpolation(0);

        CurveToThirdBlock = new Path(new BezierCurve(Score, beforeScore, new Point(-27.953, -9.6755, Point.CARTESIAN)));
        CurveToThirdBlock.setConstantHeadingInterpolation(1.497);

        CurveToScoreThird = new Path(new BezierCurve(new Point(-27.953, -9.6755, Point.CARTESIAN), beforeScore, Score));
        CurveToScoreThird.setConstantHeadingInterpolation(0);

    }

    public void loop() {
        follower.update();
        autonomousUpdate();
    }

    public void setStep(int step) {
        this.step = step;
    }

    public void autonomousUpdate() {
        switch (step) {
            case (0): {
                follower.followPath(gamePreload);
                setStep(1);
                timerReset = true;
                ScoringSlideReset = true;
                SubSlideReset = true;
                break;
            }
            case (1): {
                if (timerReset) {
                    timer.reset();
                    timerReset = false;
                }
                if (timer.milliseconds() > 0) {
                    ScoringClaw.setPosition(0.55);
                    ScoringWrist.setPosition(0.9);
                }
                if (timer.milliseconds() > 500) {
                    ScoringArm.setPosition(0.5);
                }
                if (timer.milliseconds() > 1000) {
                    if (ScoringSlideReset) {
                        moveScoringSlidesInches(1.0, 3161);
                        ScoringSlideReset = false;
                    }
                }
                if (timer.milliseconds() > 3000) {
                    follower.followPath(scorePreload);
                    timerReset = true;
                    ScoringSlideReset = true;
                    setStep(2);
                }
                break;
            }

            case (2): {
                if (timerReset) {
                    timer.reset();
                    timerReset = false;
                }

                if (timer.milliseconds() > 1000) {
                    ScoringClaw.setPosition(0.8);
                }

                if (timer.milliseconds() > 2000) {
                    follower.followPath(CurveToFirstBlock);
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

                if (timer.milliseconds() > 0) {
                    if (ScoringSlideReset) {
                        moveScoringSlidesInches(1.0, 0);
                        ScoringSlideReset = false;
                    }
                }

                if (timer.milliseconds() > 1000) {
                    if (SubSlideReset){
                        moveSubSlidesInches(1.0, 2300);
                        SubSlideReset = false;
                    }
                }

                if (timer.milliseconds() > 1500){
                    SubWrist.setPosition(0.0);
                    SubWrist.setPosition(0.66);
                }

                if (timer.milliseconds() > 2000){
                    SubClaw.setPosition(0.045);
                }
                if (timer.milliseconds() > 2500){
                    SubWrist.setPosition(0.0);
                    moveSubSlidesInches(1.0, 0);
                }

                if (timer.milliseconds() > 3000){
                    setStep(-1);
                    SubSlideReset = true;
                    ScoringSlideReset = true;
                    timerReset = true;
                }
                break;
            }
        }
    }

    public void moveSubSlidesInches(double power, int targetPosition) {

        // Reset encoder and set target position
        SubSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SubSlides.setTargetPosition(targetPosition);
        SubSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Apply power to the motor
        SubSlides.setPower(Math.abs(power));

        while (SubSlides.isBusy()) {
            telemetry.addData("Target Encoder Count", targetPosition);
            telemetry.addData("Current Position", SubSlides.getCurrentPosition());
            telemetry.update();
            TARGET_POSITION_SUB_SLIDES = SubSlides.getTargetPosition();
        }

        SubSlides.setTargetPosition((int) TARGET_POSITION_SUB_SLIDES);
        SubSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SubSlides.setPower(1.0);
    }

    public void moveScoringSlidesInches(double power, int targetPosition) {

        // Reset encoder and set target position
        ScoringSlidesMotor.setTargetPosition(targetPosition);
        ScoringSlidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Apply power to the motor
        ScoringSlidesMotor.setPower(Math.abs(power));


        while (ScoringSlidesMotor.isBusy()) {
            telemetry.addData("Target Encoder Count", targetPosition);
            telemetry.addData("Current Position", ScoringSlidesMotor.getCurrentPosition());
            telemetry.update();
            TARGET_POSITION_SCORING_SLIDES = ScoringSlidesMotor.getCurrentPosition();
        }

        ScoringSlidesMotor.setTargetPosition(TARGET_POSITION_SCORING_SLIDES);
        ScoringSlidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ScoringSlidesMotor.setPower(1.0);
    }
}
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
        ScoringSlideReset = true;
        timerReset = true;
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
        SubSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SubSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SubSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        beforeScore = new Point(2.5, -0.5, Point.CARTESIAN);
        Score = new Point(5, -0.5, Point.CARTESIAN);

        // Creating Paths
        gamePreload = new Path(new BezierCurve(new Point(0, 0, Point.CARTESIAN), beforeScore, Score));
        gamePreload.setConstantHeadingInterpolation(0);
        CurveToFirstBlock = new Path(new BezierCurve(Score, beforeScore, new Point(7.864, -4.07, Point.CARTESIAN)));
        CurveToFirstBlock.setConstantHeadingInterpolation(4.7753);

    }

    public void loop() {
        follower.update();
        telemetry.addData("Pose: ", follower.getPose());
        telemetry.addData("Heading: ", follower.getPose().getHeading());
        autonomousUpdate();
    }

    public void setStep(int step) {
        this.step = step;
    }

    public void autonomousUpdate() {
        switch (step) {
            case (0): {
                if (timerReset){
                    timer.reset();
                    ScoringWrist.setPosition(0.95);
                    ScoringArm.setPosition(1.0);
                    timerReset = false;
                }

                if (ScoringSlideReset){
                    moveScoringSlidesInches(1.0, 3161);
                    ScoringSlideReset = false;
                }
                if (timer.milliseconds() > 0){
                    ScoringWrist.setPosition(0.95);
                    ScoringClaw.setPosition(0.55);
                    ScoringArm.setPosition(1.0);
                }

                if (timer.milliseconds() > 1000) {
                    follower.followPath(gamePreload, true);
                    timerReset = true;
                    SubSlideReset = true;
                    ScoringSlideReset = true;
                    setStep(1);
                }
            }
            case (1) : {
                if (timerReset){
                    timer.reset();
                    timerReset = false;
                }
                if (gamePreload.isAtParametricEnd()){
                    ScoringClaw.setPosition(0.8);
                }
                if (timer.milliseconds() > 2000){
                    follower.followPath(CurveToFirstBlock);
                    timerReset = true;
                    SubSlideReset = true;
                    ScoringSlideReset = true;
                    setStep(2);
                }
            }
            case (2): {
                if (CurveToFirstBlock.isAtParametricEnd()){
                    if (timerReset){
                        timer.reset();
                        timerReset = false;
                    }
                    if (ScoringSlideReset){
                        moveScoringSlidesInches(1.0, 0);
                        ScoringSlideReset = false;
                    }
                    if (SubSlideReset){
                        moveSubSlidesInches(1.0, 1555);
                        SubSlideReset = false;
                    }

                    if (timer.milliseconds() > 100){
                        SubWrist.setPosition(0.66);
                        SubAngle.setPosition(0.2);
                    }
                    if (!SubSlides.isBusy()){
                        SubClaw.setPosition(0.045);
                        setStep(-1);
                    }
                }
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
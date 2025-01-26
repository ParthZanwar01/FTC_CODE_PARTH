package pedroPathing.actual_code;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "MainCode")
public class MainCode extends OpMode {

    static final double COUNTS_PER_MOTOR_REV_SubSlides = 384.5;
    static final double INCHES_PER_REV_SubSlides = 5.15;
    static final double COUNTS_PER_INCH_SubSlides = COUNTS_PER_MOTOR_REV_SubSlides / INCHES_PER_REV_SubSlides;
    DcMotor rightBack, leftBack, leftFront, rightFront, ScoringSlidesMotor, SubSlides;
    Servo ScoringWrist, ScoringArm, ScoringClaw, SubAngle, SubWrist, SubClaw;
    int targetPositionScoringSlides = 0;
    int targetPositionSubSlides = 0;

    ElapsedTime timer = new ElapsedTime();

    boolean pressed = false;
    boolean transferPressed = false;


    @Override
    public void init() {
        // Start Motors
        rightBack = hardwareMap.get(DcMotor.class, "rightRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftRear");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        ScoringWrist = hardwareMap.get(Servo.class, "ScoringWrist");
        ScoringArm = hardwareMap.get(Servo.class, "ScoringArm");
        ScoringClaw = hardwareMap.get(Servo.class, "ScoringClaw");
        SubAngle = hardwareMap.get(Servo.class, "SubAngle");
        SubWrist = hardwareMap.get(Servo.class, "SubWrist");
        SubClaw = hardwareMap.get(Servo.class, "SubClaw");
        ScoringSlidesMotor = hardwareMap.get(DcMotor.class, "ScoringSlides");
        SubSlides = hardwareMap.get(DcMotor.class, "SubSlides");

        //Set Motor Directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        //Set Slides Directions
        SubSlides.setDirection(DcMotor.Direction.REVERSE);
        ScoringSlidesMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //Set Scoring Slide Modes
        ScoringSlidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ScoringSlidesMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ScoringSlidesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*
        // Set Submersible Slide Modes
        SubSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SubSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SubSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         */
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;   // Forward/Backward (inverted)
        double x = gamepad1.left_stick_x;  // Strafe (slightly boosted for compensation)
        double rx = gamepad1.right_stick_x; // Turning Values

        // Calculate motor powers
        double frontLeftPower = y + x + rx;
        double frontRightPower = y - x - rx;
        double backLeftPower = y - x + rx;
        double backRightPower = y + x - rx;

        // Normalize powers if any value is greater than 1.0
        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        if (maxPower > 0.7) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }
        // Set motor powers
        leftFront.setPower(frontLeftPower);
        rightFront.setPower(frontRightPower);
        leftBack.setPower(backLeftPower);
        rightBack.setPower(backRightPower);


        // To grab specimen off the wall
        if (gamepad2.right_trigger > 0.1) {
            timer.reset();
            pressed = true;
        }

        if (gamepad1.options) {
            if (leftBack.getDirection() == DcMotorSimple.Direction.FORWARD) {
                leftFront.setDirection(DcMotor.Direction.REVERSE);
                rightFront.setDirection(DcMotor.Direction.FORWARD);
                leftBack.setDirection(DcMotor.Direction.REVERSE);
                rightBack.setDirection(DcMotor.Direction.FORWARD);
            } else if (leftBack.getDirection() == DcMotorSimple.Direction.REVERSE) {
                leftFront.setDirection(DcMotor.Direction.FORWARD);
                rightFront.setDirection(DcMotor.Direction.REVERSE);
                leftBack.setDirection(DcMotor.Direction.FORWARD);
                rightBack.setDirection(DcMotor.Direction.REVERSE);
            }
        }

        if (pressed) {
            if (timer.milliseconds() > 0) ScoringWrist.setPosition(1);
            if (timer.milliseconds() > 1000) ScoringArm.setPosition(0.15);
            if (timer.milliseconds() > 2000) {
                ScoringWrist.setPosition(0.04);
                pressed = false;
            }
        }

        // To score the specimen
        else if (gamepad2.left_trigger > 0.1) {
            ScoringWrist.setPosition(0.96);
            ScoringArm.setPosition(0.95);
        }

        // To close the claw
        if (gamepad2.left_bumper) {

            ScoringClaw.setPosition(0.55);
        }
        // To open the claw
        else if (gamepad2.right_bumper) {
            ScoringClaw.setPosition(0.8);
        }

        // To open the claw
        if (gamepad2.b) {
            SubClaw.setPosition(0.01);
        }

        // To close the claw
        else if (gamepad2.x) {
            SubClaw.setPosition(0.045);
        }

        // Upwards position
        if (gamepad1.left_trigger > 0.1) {
            SubWrist.setPosition(0.0);
        }
        //Downward position
        else if (gamepad1.right_trigger > 0.1) {
            SubWrist.setPosition(0.66);
        }

        // Center
        if (gamepad1.a) {
            SubAngle.setPosition(0.2);
        }

        // To the left
        else if (gamepad1.x) {
            SubAngle.setPosition(0.0);
        }
        // To the right
        else if (gamepad1.b) {
            SubAngle.setPosition(0.4);
        }

        if (gamepad2.dpad_up) { // Set power to move slides up
            ScoringSlidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ScoringSlidesMotor.setPower(1.0);
            targetPositionScoringSlides = ScoringSlidesMotor.getCurrentPosition();
            telemetry.addData("Position", targetPositionScoringSlides);
        } else if (gamepad2.dpad_down) { // Set power to move slides down
            ScoringSlidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ScoringSlidesMotor.setPower(-1.0);
            targetPositionScoringSlides = ScoringSlidesMotor.getCurrentPosition();
        } else {
            ScoringSlidesMotor.setTargetPosition(targetPositionScoringSlides);
            ScoringSlidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ScoringSlidesMotor.setPower(1.0);
        }


        if (gamepad1.dpad_up) {
            SubSlides.setPower(1.0);
        } else if (gamepad1.dpad_down) {
            SubSlides.setPower(-1.0);
        } else {
            SubSlides.setPower(0.0);
        }

        if (gamepad2.share) {
            timer.reset();
            transferPressed = true;
        }

        if (transferPressed) {
            if (timer.milliseconds() > 0 && timer.milliseconds() < 500) {
                moveSubSlidesInches(1.0, 11.5, false);
            }
            if (timer.milliseconds() > 0 && timer.milliseconds() < 3000) {
                SubAngle.setPosition(0.2);
                SubWrist.setPosition(0.33);
            }
            if (timer.milliseconds() > 500 && timer.milliseconds() < 1000) {
                SubClaw.setPosition(0.037);
            }
            if (timer.milliseconds() > 1000) {
                SubClaw.setPosition(0.045);
            }
            if (timer.milliseconds() > 2000 && timer.milliseconds() < 3000) {
                ScoringWrist.setPosition(1.0);
            }
            if (timer.milliseconds() > 2500 && timer.milliseconds() < 4000) {
                ScoringArm.setPosition(0.95);
                ScoringClaw.setPosition(0.8);
            }
            if (timer.milliseconds() > 3000) {
                ScoringWrist.setPosition(0.13);
                SubAngle.setPosition(0.85);
            }

            if (timer.milliseconds() > 3500) {
                SubWrist.setPosition(0.0);
            }
            if (timer.milliseconds() > 4000) {
                ScoringClaw.setPosition(0.55);
            }
            if (timer.milliseconds() > 4500) {
                SubClaw.setPosition(0.0);
                ScoringWrist.setPosition(0.96);
            }
            if (timer.milliseconds() > 5000) {
                moveScoringSlidesInches(1.0, 3161);
                transferPressed = false;
            }
        }
    }

    public void moveSubSlidesInches(double power, double inches, boolean hold) {
        int targetPosition = (int) (inches * COUNTS_PER_INCH_SubSlides);

        // Reset encoder and set target position
        SubSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SubSlides.setTargetPosition(targetPosition);
        SubSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Apply power to the motor
        SubSlides.setPower(Math.abs(power));

        // Wait until the motor reaches the target position or timeout occurs


        while (SubSlides.isBusy()) {
            telemetry.addData("Target Position (Inches)", inches);
            telemetry.addData("Target Encoder Count", targetPosition);
            telemetry.addData("Current Position", SubSlides.getCurrentPosition());
            telemetry.update();
        }

        // Stop motor
        SubSlides.setPower(0);

        // Optionally hold position
        if (hold) {
            SubSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            SubSlides.setPower(0.02); // Small power to counteract gravity
        } else {
            SubSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
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
            targetPositionScoringSlides = ScoringSlidesMotor.getCurrentPosition();
        }

        ScoringSlidesMotor.setTargetPosition(targetPositionScoringSlides);
        ScoringSlidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ScoringSlidesMotor.setPower(1.0);
    }

}
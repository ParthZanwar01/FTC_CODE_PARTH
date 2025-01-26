package pedroPathing.actual_code;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


// Read the following!!!
// PLug in the servo into the 5th port of the control hub
@TeleOp(name = "ZeroOutServo")
public class ZeroOutServo extends OpMode {

    public Servo ScoringWrist;
    public Servo ScoringArm;
    public Servo ScoringClaw;
    public Servo SubAngle;
    public Servo SubWrist;
    public Servo SubClaw;

    @Override
    public void init() {
        ScoringWrist = hardwareMap.get(Servo.class, "ScoringWrist");
        ScoringArm = hardwareMap.get(Servo.class, "ScoringArm");
        ScoringClaw = hardwareMap.get(Servo.class, "ScoringClaw");
        SubAngle = hardwareMap.get(Servo.class, "SubAngle");
        SubWrist = hardwareMap.get(Servo.class, "SubWrist");
        SubClaw = hardwareMap.get(Servo.class, "SubClaw");
    }

    @Override
    public void loop() {
        //Zeroes out the servo
        if (gamepad1.left_bumper){
            SubClaw.setPosition(0.045);
        }
        else if (gamepad1.right_bumper){
            SubClaw.setPosition(0.0);
        }
    }
}

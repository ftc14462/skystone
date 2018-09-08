import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class MyFIRSTJavaOpMode extends LinearOpMode {

    private Gyroscope imu;
    private DcMotor motorTest;
    private DigitalChannel digitalTouch;
    private DistanceSensor sensorColorRange;
    private Servo servoTest;

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(Gyroscope.class, "imu");
        motorTest = hardwareMap.get(DcMotor.class, "test motor");
        digitalTouch = hardwareMap.get(DigitalChannel.class, "digital touch");
        sensorColorRange = hardwareMap.get(DistanceSensor.class, "test sensor");
        servoTest = hardwareMap.get(Servo.class, "test servo");
        // set digital channel to input mode.
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        double tgtPower = 0;
        double tgtPos = 0;

        while (opModeIsActive()) {
            tgtPower = -this.gamepad1.left_stick_y;
            motorTest.setPower(tgtPower);

            /*//check to see if we need to move the servo.
            if (gamepad1.y) {
            // move to 0 degrees.
                servoTest.setPosition(0);
            } else if (gamepad1.x || gamepad1.b) {
            // move to 90 degrees.
                servoTest.setPosition(0.5);
            } else if (gamepad1.a) {
            // move to 180 degrees.
                servoTest.setPosition(1);
            }*/

            //set servo to stick x position
            tgtPos =  0.5 - this.gamepad1.right_stick_y*0.5;
            servoTest.setPosition(tgtPos);

            telemetry.addData("Target Position", tgtPos);
            telemetry.addData("Servo Position", servoTest.getPosition());
            telemetry.addData("Target Power", tgtPower);
            telemetry.addData("Motor Power", motorTest.getPower());
            // is button pressed?
            if (digitalTouch.getState() == false) {
            // button is pressed.
                telemetry.addData("Button", "PRESSED");
            }
            else {
                // button is not pressed.
                telemetry.addData("Button", "NOT PRESSED");
            }
            telemetry.addData("Distance (cm)", sensorColorRange.getDistance(DistanceUnit.CM));
            //telemetry.addData("imu velocity", imu.getAngularVelocity(AngleUnit.DEGREES));
            //telemetry.addData("imu velocity axis (0=x, 1=y, 2=z)", imu.getAngularVelocityAxes());
            telemetry.addData("Status", "Running");

            telemetry.update();
        }
    }
}
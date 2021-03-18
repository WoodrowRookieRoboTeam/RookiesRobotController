package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */



@TeleOp(name="Basic Drive: Iterative OpMode", group="Iterative Opmode")
public class BaseDrive extends OpMode
{
    //public enum botState { off, running}

    final int MAX_LIFT_ENCODER_SETTING = 4945;

    //botState curBotState = botState.off;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;

    private DcMotor liftMotor = null;
    private Servo clawRotation = null;
    private Servo clawOpen = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightDrive  = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightDrive  = hardwareMap.get(DcMotor.class, "backRightMotor");

        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        clawRotation = hardwareMap.get(Servo.class, "clawRotation");
        clawOpen = hardwareMap.get(Servo.class, "clawOpen");


        liftMotor.setTargetPosition(3200);


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode((DcMotor.RunMode.RUN_USING_ENCODER));

        clawRotation.setPosition(1);
        clawOpen.setPosition(0.5);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double frontLeftPower, frontRightPower, backLeftPower, backRightPower;

        /// Outdated Calculation
        //double angle = Math.toDegrees(-Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) + Math.PI/2);
        //double mag = Math.sqrt(Math.pow(gamepad1.right_stick_y, 2) + (Math.pow(gamepad1.right_stick_x, 2)));

        //Binds directions for each joystick
        //All robot movement is controlled by gamepad1
        double y = gamepad1.left_stick_y; // reversed y for axis
        double x = -gamepad1.left_stick_x * 1.5;
        double rx = -gamepad1.right_stick_x;

        //Sets values for each wheel motor
        frontLeftPower = y + x + rx;
        frontRightPower = y - x - rx;
        backLeftPower = y - x + rx;
        backRightPower = y + x - rx;

        // All claw movement is controlled by gamepad2

        //opening and closing of claw
        if (gamepad1.a){
            clawOpen.setPosition(0.9);
        }
        else{
            clawOpen.setPosition(0.5);
        }

        //Rotates claw 90 degrees
        if (gamepad1.x) {
            clawRotation.setPosition(0.28);
        }
        //Sets claw back to initial rotation
        else if (gamepad1.b){
            clawRotation.setPosition(1);
        }

        //Continuous lift system
        if (gamepad1.dpad_up && liftMotor.getCurrentPosition() < MAX_LIFT_ENCODER_SETTING){
            liftMotor.setPower(0.5);
        }
        else{
            liftMotor.setPower(0);
        }

        if (gamepad1.dpad_down && liftMotor.getCurrentPosition() > 0){
            liftMotor.setPower(-0.5);
        }
        else{
            liftMotor.setPower(0);
        }

        //Controls height of claw


        // Show the elapsed game time and wheel power.
        /*
        telemetry.addData("Elapsed Time", runtime);
        telemetry.addData("LX", gamepad1.left_stick_x);
        telemetry.addData("RX", gamepad1.right_stick_x);
        telemetry.addData("RY", gamepad1.right_stick_y);
        telemetry.addData("FrontLeftPower", frontLeftPower);
        telemetry.addData("FrontRightPower", frontRightPower);
        telemetry.addData("BackLeftPower", backLeftPower);
        telemetry.addData("BackRightPower", backRightPower);*/
        telemetry.addData("clawRotation", clawRotation.getPosition());
        telemetry.addData("clawOpen", clawOpen.getPosition());
        telemetry.addData("liftPower", liftMotor.getPower());
        telemetry.addData("lift encoder", liftMotor.getCurrentPosition());
        telemetry.addData("isB", gamepad1.b);


        // calculate maximum value to divide by
        if (Math.abs(frontLeftPower) > 1 || Math.abs(frontRightPower) > 1 || Math.abs(backLeftPower) > 1 || Math.abs(backRightPower) > 1){
            double max = 0;
            max = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
            max = Math.max(Math.abs(frontRightPower), max);
            max = Math.max(Math.abs(backRightPower), max);

            // divide everything by max
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Send calculated power to wheels
        frontLeftDrive.setPower(y + x + rx);
        frontRightDrive.setPower(y - x - rx);
        backLeftDrive.setPower(y - x + rx);
        backRightDrive.setPower(y + x - rx);




    }

    /*private double getMechanamMotorPower(float right_stick_y, String m) {

        if(m.equals(("FL")) {
            return the FL motor setting for the directyion right_stick_y
        }



    }*/

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}

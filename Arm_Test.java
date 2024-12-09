package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Arm Test", group="2024-2025")
public class Arm_Test extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

	//Declare Motors and Servos
    public DcMotor BL_Motor = null;
    public DcMotor BR_Motor = null;
    public DcMotor FL_Motor = null;
    public DcMotor FR_Motor = null;

	public DcMotor Rack_Motor = null;
	public double Rack_Pos = 0.0;
	public double Rack_Upper = 0.0;
	public double Rack_Lower = 0.0;
	public double Rack_Pad = 0.0;

	public DcMotor Arm_Motor = null;
	public double Arm_Pos = 0.0;
	public double Arm_Upper = 0.0;
	public double Arm_Lower = 0.0;
	public double Arm_Pad = 0.0;

    public Servo Hand_Servo = null;

    public double Rack_inc = 0.05;
    public boolean dpad_latch_right = false;
    public boolean dpad_latch_left = false;


    @Override
    public void runOpMode() {
	// Map Motors and Servos
        BR_Motor = hardwareMap.get(DcMotor.class, "BR Wheel Motor");
        FL_Motor = hardwareMap.get(DcMotor.class, "FL Wheel Motor");
        BL_Motor = hardwareMap.get(DcMotor.class, "BL Wheel Motor");
        FR_Motor = hardwareMap.get(DcMotor.class, "FR Wheel Motor");

		Rack_Motor = hardwareMap.get(DcMotor.class, "Rack Motor");
		Rack_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
		Rack_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); 
		
	    Arm_Motor = hardwareMap.get(DcMotor.class, "Arm Motor");
		Arm_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
		Arm_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); 

        Hand_Servo = hardwareMap.get(Servo.class, "Hand Servo");

		double max = 0.0;

        double axial   = 0.0;
        double lateral = 0.0;
        double yaw     = 0.0;

	double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

		//Drive Test
        //FL_Motor.setDirection(DcMotor.Direction.REVERSE);
        //BL_Motor.setDirection(DcMotor.Direction.REVERSE);
        //FR_Motor.setDirection(DcMotor.Direction.FORWARD);
        //BR_Motor.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

			//Update Sensors
			Rack_Pos = Rack_Motor.getCurrentPosition();
			Arm_Pos = Arm_Motor.getCurrentPosition();
			
        	axial   = -gamepad1.left_stick_y;
        	lateral =  gamepad1.left_stick_x;
        	yaw     =  gamepad1.right_stick_x;
			
            leftFrontPower  = axial + lateral + yaw;
            rightFrontPower = axial - lateral - yaw;
            leftBackPower   = axial - lateral + yaw;
            rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower),Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // Send calculated power to wheels
            FL_Motor.setPower(leftFrontPower);
            FR_Motor.setPower(rightFrontPower);
            BL_Motor.setPower(leftBackPower);
            BR_Motor.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Rack_Pos", "%4.2f", Rack_Pos);
			telemetry.addData("Arm_Pos", "%4.2f", Arm_Pos);

            telemetry.update();
        }
	
    }}


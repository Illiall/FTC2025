package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;

@TeleOp
public class Main extends LinearOpMode {

    @Override
    public void runOpMode(){
        //Creates a new mecanum drivetrain called dt (drivetrain)
        MecanumDrive dt = new MecanumDrive(hardwareMap);
        //Creates a new intake from the subsystem Intake
        //Intake intake = new Intake(hardwareMap);
        //Creates a new shooter from the subsystem Shooter
        //Shooter shooter = new Shooter(hardwareMap);

        //Waits for the Robot Controller to start
        waitForStart();
        //Runs the code below if the op mode is on
        while(opModeIsActive()){
            //Gamepad joysticks
            //Gets values from the gamepad for the values used in the mecanum drive train
            //Gets a double forward value
            double forwards = gamepad1.left_stick_y;
            //Gets a double strafe Value
            double strafe = gamepad1.left_stick_x;
            //Gets a double rotation Value
            double rotation = gamepad1.right_stick_x;

            //Gamepad buttons
            //gamepad 1
            //Boolean returns True or False

            //Left bumper resets the robots yaw
            boolean reset_yaw = gamepad1.left_bumper;

            //gamepad 2
            //left bumper is intake
            boolean in = gamepad2.left_bumper;
            //right bumper is stop intake
            boolean stop_intake = gamepad2.right_bumper;
            //button x is to shoot
            boolean shoot = gamepad2.x;

            //Drivetrain
            //Uses the values gotten from the gamepads for the drive train
            dt.field_relative(forwards, strafe, rotation);
            //Resets the Yaw on the robot
            if(reset_yaw){dt.reset_yaw();}

            //Intake
            //If boolean variable "in" is True
            /*if(in){
                //intake
                intake.intake();
                //if stop_intake is True
            }else if(stop_intake) {
                //Stops the intake |!For Emergencies!|
                intake.stop();
                //Otherwise if "in" is False
            }else{
                //Hold the game piece
                intake.hold();
            }

            //Shooter
            //If boolean variable "shoot" is True
            if(shoot){
                //Shoot
                shooter.shoot();
                //Otherwise if "shoot" is False
            }else{
                //Stop
                shooter.stop();
            }*/

            telemetry.addData("Forwards power", forwards);
            telemetry.addData("Strafe power", strafe);
            telemetry.addData("Rotation power", rotation);
            telemetry.addData("Yaw in Radians",dt.getYaw());
            telemetry.addData("Max power", constants.max_drive_power);
            telemetry.addData("Max speed", constants.max_drive_speed);
            telemetry.update();
        }
    }
}

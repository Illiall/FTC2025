package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.constants;

public class MecanumDrive {
    //Defining motors
    DcMotor fr_motor, fl_motor, br_motor, bl_motor;
    //Defines the IMU
    IMU imu;
    //creates a variable for the orientation of the robot for the IMU
    RevHubOrientationOnRobot orientation;
    //Create multiple variable for the robot
    double theta,new_theta, distance, max_power, max_speed, fwds, strf, fr_power, fl_power, br_power, bl_power;
    //Initializes the parts on the robot
    public MecanumDrive(HardwareMap hardwaremap) {
        //Sets the motor names
        fr_motor = hardwaremap.get(DcMotor.class, "FR");
        fl_motor = hardwaremap.get(DcMotor.class, "FL");
        br_motor = hardwaremap.get(DcMotor.class, "BR");
        bl_motor = hardwaremap.get(DcMotor.class, "BL");
        //Sets the IMU name
        imu = hardwaremap.get(IMU.class,"imu");
        //Sets the orientation on the of the imu on the robot
        //IMPORTANT: IF CONTROL HUB CHANGES ORIENTATION CHANGE THE ORIENTATION IN THE CODE
        orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        //Reverse motors if needed
        fr_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        br_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        //fl_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        //bl_motor.setDirection(DcMotorSimple.Direction.REVERSE);

        //initialize imu
        imu.initialize(new IMU.Parameters(orientation));
    }
    public void reset_yaw(){imu.resetYaw();}
    //function that gets the yaw of the robot in radians with the IMU
    public double getYaw(){return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);}
    //Creates a function that takes 3 values from field relative to set the power on the motors
    public void robot_relative(double forwards, double strafe, double rotation){
        //calculates basic power requirements for the motors
        fl_power = forwards + strafe + rotation;
        bl_power = forwards - strafe + rotation;
        fr_power = forwards - strafe - rotation;
        br_power = forwards + strafe - rotation;

        //constants to limit motor power and speed
        max_power = constants.max_drive_power;
        max_speed = constants.max_drive_speed;

        //Maximizes the power for the motors
        max_power = Math.max(max_power, Math.abs(fl_power));
        max_power = Math.max(max_power, Math.abs(bl_power));
        max_power = Math.max(max_power, Math.abs(fr_power));
        max_power = Math.max(max_power, Math.abs(br_power));

        //Sets the motors power so that it doesn't go over max speed and max power
        fl_motor.setPower(max_speed * (fl_power/max_power));
        fr_motor.setPower(max_speed * (fr_power)/max_power);
        bl_motor.setPower(max_speed * (bl_power)/max_power);
        br_motor.setPower(max_speed * (br_power)/max_power);
    }
    /*Creates a function that takes the gamepad values changes them, then uses the values
     in the robot relative driving*/
    public void field_relative(double forwards, double strafe, double rotation){
        //calculates the angle with forwards and strafe
        theta = Math.atan2(forwards,strafe);
        //calculates the distance with forwards and strafe
        distance = Math.hypot(strafe,forwards);
        //calculates the current rotation
        new_theta = AngleUnit.normalizeRadians(theta - getYaw());
        //calculates the new forwards power and new strafe power
        fwds = distance * Math.sin(new_theta);
        strf = distance * Math.cos(new_theta);
        //Uses the calculated values for the input for the robot relative drive
        this.robot_relative(fwds,strf,rotation);
    }
}

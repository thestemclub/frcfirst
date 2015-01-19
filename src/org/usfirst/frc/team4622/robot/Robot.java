
package org.usfirst.frc.team4622.robot;


import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;

/**
 * This is a demo program showing the use of the RobotDrive class.
 * The SampleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 *
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SampleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 *
 * WARNING: While it may look like a good choice to use for your code if you're inexperienced,
 * don't. Unless you know what you are doing, complex code will be much more difficult under
 * this system. Use IterativeRobot or Command-Based instead if you're new.
 */

public class Robot extends SampleRobot {
    RobotDrive driveSystem;
    
    Joystick stick;
    
    Joystick stick2;
    
    AnalogPotentiometer po1;
    AnalogPotentiometer po2;
    
    Servo hor;
    Servo ver;
    
    Jaguar backRight; //Port 1
    
    Jaguar backLeft; //Port 2
    
    Jaguar frontLeft; //Port 3
    
    Jaguar frontRight; //Port 4
    
    double LX;
    
    double horPos;
    double verPos;
    
    Gyro gyro;   
    Gyro armGyro;
    
    double sens;
    
    SmartDashboard dash;
    
    CameraServer server;
    
    Talon test;
    
    AnalogInput ultrasonic1;
    
    public Robot() {
    	frontLeft = new Jaguar(5);
        frontRight = new Jaguar(4);
        backRight = new Jaguar(1);
        backLeft = new Jaguar(2);
        
        stick = new Joystick(0);
        stick2 = new Joystick(1);
        
        gyro = new Gyro(0);
        gyro.startLiveWindowMode();
        
        armGyro = new Gyro(1);
        armGyro.startLiveWindowMode();
        
        po1 = new AnalogPotentiometer(9);
        po1 = new AnalogPotentiometer(8);
        
        hor = new Servo(7);
        ver = new Servo(6);
        
        dash = new SmartDashboard();

        server = CameraServer.getInstance();
        server.setQuality(50);
        //the camera name (ex "cam0") can be found through the roborio web interface
        server.startAutomaticCapture("cam0");
        
        // USE JAGUAR CLASS TO CONTROL TALONS
        test = new Talon(0);
        
        ultrasonic1 = new AnalogInput(2);
        
        sens = 4;
        
    }

    /**
     * Drive left & right motors for 2 seconds then stop
     */
    public void autonomous() {
        
    }

    /**
     * Runs the motors with arcade steering.
     */
    public void operatorControl() {
        //driveSystem.setSafetyEnabled(true);
        driveSystem = new RobotDrive(frontLeft, backLeft, frontRight, backRight);
        
        horPos = 90;
        verPos = 45;
        
        driveSystem.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
        driveSystem.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);
        
        //gyro.reset();
        //gyro.setSensitivity(sens);
        //gyro.initGyro();
        
        //armGyro.reset();
        //armGyro.setSensitivity(sens);
        //armGyro.initGyro();
        
        while (isOperatorControl() && isEnabled()) {
            Timer.delay(0.005);
            
            driveSystem.mecanumDrive_Cartesian(stick.getRawAxis(0), -stick.getRawAxis(1), -stick.getRawAxis(4), gyro.getAngle());
            
            if(stick2.getRawAxis(0) + .2 < 0){
            	if(horPos > 0){
            		horPos--;
            	}
            }else if(stick2.getRawAxis(0) - .2 > 0){
            	if(horPos < 179){
            		horPos++;
            	}
            }else{
            	
            }
            
            if(stick2.getRawAxis(1) + .2 < 0){
            	if(verPos < 179){
            		verPos++;
            	}
            }else if(stick2.getRawAxis(1) - .2 > 0){
            	if(verPos > 0){
            		verPos--;
            	}
            }else{
            	
            }
            
            if(armGyro.getAngle() - .2 < 0){
            	test.set(-1.0);
            }else if(armGyro.getAngle() + .2 > 0){
            	test.set(1.0);
            }
            
            if(stick2.getRawButton(1)){
            	verPos = 45; //45
            	horPos = 90; //90
            	
            	
            }
            
            SmartDashboard.putNumber("ver", verPos);
            SmartDashboard.putNumber("hor", horPos);
            SmartDashboard.putNumber("STICK", stick2.getRawAxis(1));
            
            
            gyro.updateTable(); //send updated gyro values
            armGyro.updateTable();
            
            SmartDashboard.putNumber("arm gyro", armGyro.getAngle());
            SmartDashboard.putNumber("arm gyro rate", armGyro.getRate());
            SmartDashboard.putNumber("gyro", gyro.getAngle());
            SmartDashboard.putNumber("gyroConverted", convertGyroAngle(gyro.getAngle()));
            SmartDashboard.putNumber("gyro rate", gyro.getRate());
            hor.setAngle(horPos);
            ver.setAngle(verPos);
            SmartDashboard.putNumber("ultrasonic1",ultrasonic1.getAverageVoltage());
            armCorrect();
        }
    }

    /**
     * Runs during test mode
     */
    public void test() {
    }
    
    public static double convertGyroAngle(double rawGyroAngle){
    	while(rawGyroAngle < 0){
    		rawGyroAngle += 360.00;
    	}
    	while(rawGyroAngle > 360){
    		rawGyroAngle -= 360.00;
    	}
    	return rawGyroAngle;
    }
    
    public void armCorrect(){
    	if(po2.get() < 180 - po1.get()){
    		
    	}else if(po2.get() > 180 - po1.get()){
    		
    	}
    }
}

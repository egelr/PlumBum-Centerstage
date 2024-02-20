package org.firstinspires.ftc.teamcode.util;

public class variables {
    static final boolean FIELD_CENTRIC = false;
    //Arm Motor 1 (turning) speed
    public static final double sp = 0.05;
    //Arm Motor 2 (extending) speed
    //public static final double sp2 = 0.05;
    //Arm Motor 2 (extending) speed
    public static final double speed_extender = 0.1;
    //Arm Motor 1 (turning) speed
    public static final double speed_arm = 0.15;
    //Claw arm angles for closing the LeftServo and opening the RightServo
    public static final double gripDegrees = 43;
    //Claw arm angles for closing the RightServo and opening the LeftServo
    public static final double gripDegrees1 = 82;

    //Arm turning position for the Backboard
    public static final int nearBoard = 2950;
    //Arm extending position for pixel grabbing
    public static final int reachingHigh = 1350;
    //Arm extending position for near the Backboard
    public static final int reachingHigh_Board = 1000;
    //Arm motors working cycle time limit
    public static final int timer_motor = 1;
    public static final int timer_motor_2 = 1;
    //Claw angle for Grabbing pixels (parallel with the Ground)
    public static final int ClawAngleGrab = 76;
    //Claw angle for Depositing pixels (parallel with the Backboard)
    public static final int ClawAngleDeposit = 85;
    //Simple-auto Claw angles for pixel depositing and going back to park
    public static final double AutoCLawDown = 70;
    public static final double AutoCLawPark = 0;
    //Auto positions (depending on where the prop is)
    public static final int NOTDETECTED = 0;
    public static final int LEFT = 1;
    public static final int CENTRE = 2;
    public static final int RIGHT = 3;
    //Auto trajectory when the prop is in the center
    public static final double Center1x = 30;
    public static final double Center1y = 0;
    //Auto trajectory when the prop is in the left (for RedNear and BlueFar) and in the right (for RedFar and BlueNear)
    public static final double Obs11x = 30;
    public static final double Obs11y = 5;
    public static final double Obs11Forward = 5;
    public static final double LeftLeft = 12;
    public static final double LeftForward = 8;
    //Auto trajectory when the prop is in the right (for RedNear and BlueFar) and in the left (for RedFar and BlueNear)
    public static final double Free11x = 23;
    public static final double Free11y = 11;
    public static final double Near2y = 32;
    public static final double Far2y = 80;
    //Auto trajectory when the prop is not found (Near)
    public static final double Near2xCenter = 29;
    public static final double NearBoard = 8;
    public static final double Park1Center = 24;
    public static final double Park2 = 10;
    public static final double FarNF01 = 50;
    public static final double FarNF02 = 70;
    //Auto trajectory when the prop is not found (Far)
    public static final double NFBackFar = 65;
    public static final double NFForwardFar = 100;
    public static final double NFBackFar1 = 60;
    public static final double RightLeft1 = 22;






}
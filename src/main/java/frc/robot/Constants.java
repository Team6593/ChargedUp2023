// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Sort out ID's by classes
    // MAKE SURE THE CLASSES ARE DECLARED STATIC
    // ALSO MAKE SURE TO IMPORT INNER-CLASSES STATICALLY

    public final int XboxController_Port = 0;

    public static final class CameraVision {
        public final String cameraName = "camera";
        public final int ViewportWidth = 640;
        public final int ViewportHeight = 480;
    }
    public static final class LimitSwitchesPorts{

        public final int ElevatorTopLimitSwitchPort = 0;
        public final int ElevatorLowLimitSwitchPort = 1;

        // // Arm limit switches (WIP)
        public final static int ArmLimitSwitchTop = 0;
        public final static int ArmLimitSwitchBottom = 0;
    }

    public static final class InputMap {
    
        public static final class xBox{
            public final int Abutton = 1;
            public final int Bbutton = 2;
            public final int Xbutton = 3;
            public final int Ybutton = 4;
            public final int LeftTrigger = 5;
            public final int RightTrigger = 6;
            public final int WindowButton = 7;
            public final int MenuButton = 8;
            public final int LeftButtonClick = 9;
            public final int RightButtonClick = 10;
        }
    }

    public static final class Motors {

        public static final int ArmReelerMotorID = 0;
        public static final int ArmMotorID = 0;
        // put motor ID's here
        // how to define consts: subsystem + master or follower + orientation (left, right, up, down) + "ID"
        // change these nums later
        public final int MasterRight = 1;
        public final int FollowerRight = 2;
        public final int MasterLeft = 3;
        public final int FollowerLeft = 4;
        public final double FalconUnitsPerRevolution = 2048;

        // Reeler motors
        public final int TopMotorID = 10;
        public final int BottomMotorID = 11;

        // Elevator motor
        public final int ElevatorMotorID = 0;

        //units per revolution of talon Motors
        public final int TalonFX_UnitsPerRev = 2048;
    }

    public static final class SpeedsForMotors{
        public final double Elevator_setSpeed = 0.2;
    }

    public static final class DoubleSolenoidChannels{
        // DriveTrain
        public static final int ForwardChannelDt = 4;
        public static final int ReverseChannelDt = 5;

        // Arm
        public final int ArmCloseChannel = 0;
        public final int ArmOpenChannel = 0;
        public final int ArmExtendChannel = 0;
        public final int armRetractChannel = 0;

    }
    public static final class Autonomous {
        public final double encoderDistanceToChargeStation = 1223.760000; //couple feet or so, change later
        public final double balancingSpeed = .15; // 15 percent speed
        public final double rollThreshholdDegrees = 1;
    }

    public static final class PIDValues{
        public final double KPARM = .01;
    }

}
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
    public final int ButtonBoard_Port = 1;

    public static final class CameraVision {
        public final String cameraName = "camera";
        public final int ViewportWidth = 640;
        public final int ViewportHeight = 480;
    }
    
    public static final class LimitSwitchesPorts{

        public final int ElevatorTopLimitSwitchPort = 0;
        public final int ElevatorLowLimitSwitchPort = 1;

        // // Arm limit switches (WIP)
        // CHANGE LATER
        public final int ArmLimitSwitchTop = 3;
        public final int ArmLimitSwitchBottom = 2;
    }

    public static final class InputMap {
        
        public static final class ButtonBoard {
           public final int ArmExtend = 1;
           public final int ArmRetract = 2;
           public final int StartingConfig = 3;
           public final int FloorPickup = 4;
           public final int HumanStation = 5;
           public final int ScoringMid = 6;
           public final int grab = 7;
           public final int release = 8;

           public final int disable = 9;
           public final int EmergencyStop = 10;
        }

        public static final class xBox{
            public final int Abutton = 1;
            public final int Bbutton = 2;
            public final int Xbutton = 3;
            public final int Ybutton = 4;
            public final int LeftTrigger = 5;
            public final int RightTrigger = 6;
            public final int WindowButton = 7;
            public final int MenuButton = 8;
            public final int EmergencyStop = 9;
            public final int LeftButtonClick = 9;
            public final int RightButtonClick = 10;
        }
    }

    public static final class Motors {

        public final int ArmReelerMotorID = 0; // do not use
        public final int ArmMotorID = 9;
        // put motor ID's here
        // how to define consts: subsystem + master or follower + orientation (left, right, up, down) + "ID"
        // change these nums later
        public final int MasterRight = 1;
        public final int FollowerRight = 2;
        public final int MasterLeft = 3;
        public final int FollowerLeft = 4;
        public final double FalconUnitsPerRevolution = 2048;

        // Reeler motors
        public final int TopMotorID = 14;
        public final int BottomMotorID = 9;

        // Elevator motor
        public final int ElevatorMotorID = 11;

        //units per revolution of talon Motors
        public final int TalonFX_UnitsPerRev = 2048;
    }

    public static final class SpeedsForMotors{
        //speeds for arm and reeler
        public final double ArmSpeed = 0;
        public final double ReelerSpeed = 0;

        //speed for elevator
        public final double ElevatorSpeed = 0.5;

        // arm
        
    }

    public static final class DoubleSolenoidChannels{
        // DriveTrain
        // swap values?
        public final int ForwardChannelDt = 0;
        public final int ReverseChannelDt = 1;

        // Arm
        // swap values? unsure values.
        public final int ArmCloseChannel = 2;
        public final int ArmOpenChannel = 3;

        // swap values?,these are open and close (e: 2, r: 3)?
        public final int ArmExtendChannel = 7;
        public final int ArmRetractChannel = 6;

    }
    public static final class Autonomous {
        public final double encoderDistanceToChargeStation = 1223.760000; //couple feet or so, change later
        public final double levelDegrees = 71;
        public final double limelightLensHeightInches = 50.5;
        public final double limelightMountAngleDegrees = 10;
        public final double midPegHeight = 24 + 10; // 2 feet 10 inches
        public final double testHeight = 26;
    }

    public static final class PIDValues{
        public final double KPARM = .01;
    }

}
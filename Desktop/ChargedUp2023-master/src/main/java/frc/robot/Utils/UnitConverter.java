// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

/** Unit conversion class, converts between radians, degrees and units */
public class UnitConverter {

    /**
     * Determines the number of encoder units needed to perform a certain amount of rotations needed
     * based on the formula: Units-Per-Revolution * gear-ratio * rotations-needed
     * @param gearRatio ask mechanical (10:1 -> 10/1)
     * @param unitsPerRevolution - how many units per revolution (TalonFX uses 2048)
     * @param rotationsNeeded - the amount of rotations needed
     * @return encoder units needed to perform a certain amount of rotations needed
     */
    public double determineUnitsPerRevolution(double gearRatio, double unitsPerRevolution, int rotationsNeeded) {
        // type cast rotationsNeeded?
        return unitsPerRevolution * gearRatio * rotationsNeeded;
    }
    //determines distance driven in inches
    /**
     * determines distance driven when a number of revolutions are given
     * @param diameterOfWheel (measure of straight line across the center of the wheel)
     * @param unitsPerRevs (revolutions given to measure the distance that will be driven)
     * @return
     */
    public double distanceToTravelCalcultions(double diameterOfWheel, double unitsPerRevs, double distanceInInches){
        double circumferenceOfWheel = diameterOfWheel * Math.PI;//PI is the radius of the wheel(circumference to diameter)
        double distancePerRevs = unitsPerRevs * circumferenceOfWheel;
        double driveDistance = distancePerRevs * distanceInInches;

        return driveDistance;
    }
    /**
     * converts raw TalonFX integrated-sensor units into RECU, or readable-encoder-units
     * the formula is r/10 = RECU, where r is the raw units
     * @param rawUnits
     * @return readable-encoder-units
     */
    public double toReadableEncoderUnit(double rawUnits) {
        double recu = rawUnits /100;
        return recu;
    }
    /**
     * converts actual units per revolution to number 
     * that matches with the readable encoder values 
     * @param motorUPR (motor-units-per-revolution)
     * @return unitsPerRev
     */
    public double unitsPerRev_toMatchREU(double motorUPR){
        double unitsPerRev = motorUPR / 100;//divide by number the encoder's raw units are being divided by
        return unitsPerRev;
    }

    /**
     * An ideal use-case for this is when you are programming sensors/encoders/autonomoue
     * Converts degrees into radians
     * @param degrees
     * @return radians
     */
    public double deg2rad(double degrees) {
        double radians = Math.toRadians(degrees);
        return radians;
    }

    /**
     * An ideal use-case for this is when you are programming sensors/encoders/autonomoue
     * Converts radians into degrees
     * @param radians
     * @return degrees
     */
    public double rad2deg(double radians) {
        double degrees = Math.toDegrees(radians);
        return degrees;
    }

    /**
     * An ideal use-case for this is when you are programming sensors/encoders/autonomoue
     * @param units
     * @return units per revolution/full rotation (360 degrees)
     */
    public double unitsPerFullRevolution(double units) {
        double unitsPerRev = units / 360;
        return unitsPerRev;
    }
    
    /**
     * An ideal use-case for this is when you are programming sensors/encoders/autonomoue
     * @param units
     * @return units per half-revolution/half of a full rotation (180 degrees)
     */
    public double unitsPerHalfRevolution(double units) {
        double unitsPerHalfRev = units / 180;
        return unitsPerHalfRev;
    }

    /**
     * An ideal use-case for this is when you are programming sensors/encoders/autonomoue
     * @param units
     * @param degrees
     * @return units per-degree (units/degree)
     */
    public double unitsPerDegree(double units, double degrees) {
        double unitsPerDegr = units / degrees;
        return unitsPerDegr;
    }

    /**
     * An ideal use-case for this is when you are programming sensors/encoders/autonomoue
     * @param degree
     * @param units
     * @return degree(s) per unit (degree/units)
     */
    public double degreePerUnits(double degree, double units) {
        double degPerUnit = degree / units;
        return degPerUnit;
    }
    
}

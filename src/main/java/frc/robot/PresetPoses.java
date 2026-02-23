package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class PresetPoses {
    public Pose2d goalPose = new Pose2d(4.621, 4.029, new Rotation2d(0));

    public static double[][] powerTable = new double[][] {
            {44.24, 0.445},
            {56.74, 0.46},
            {73.0619, 0.475},
            {80.48, 0.5},
            {90.8514, 0.525},
            {112.78, 0.595},
            {137.9374, 0.68}
    };

    public PresetPoses(boolean isRed) {
        if (isRed) {
            goalPose = mirrorPose(goalPose);
        }
    }

    /**
     * Mirrors poses across alliances for interchangeability from either side of the field. This is done by mirroring the x coordinate across the center line of the field and adjusting the rotation accordingly.
     * @param pose The pose to be mirrored.
     * @return The mirrored pose.
     */
    public Pose2d mirrorPose(Pose2d pose) {
        double fieldWidth = 8.229; // Width of the field in meters
        double mirroredX = fieldWidth - pose.getX();
        double mirroredRotation = Math.toRadians((pose.getRotation().getDegrees() - 180) % 360); // Mirror the rotation
        return new Pose2d(mirroredX, pose.getY(), new Rotation2d(mirroredRotation));
    }

    public double getAngleToGoalRadians(Pose2d currentPose) {
        Translation2d toGoal = goalPose.getTranslation().minus(currentPose.getTranslation());
        return Math.atan2(toGoal.getY(), toGoal.getX());
    } 

    public double getOptimalShooterPowerPercentage(Pose2d currentPose, boolean doLinearInterp) {
        double distance = currentPose.getTranslation().getDistance(goalPose.getTranslation()); // in inches (pedropathing coords are in inches)
        // clamp to least and highest indexes
        if (distance <= powerTable[0][0]) return powerTable[0][1];
        if (distance >= powerTable[powerTable.length - 1][0]) return powerTable[powerTable.length - 1][1];



        if (!doLinearInterp) {
            int optimalCoordinateIndex = 0;
            double minDiff = Double.MAX_VALUE;
            for (int i = 0; i < powerTable.length; i++) {
                double coorDist = powerTable[i][0];
                if (minDiff > Math.abs(coorDist - distance)) {
                    optimalCoordinateIndex = i;
                }
            }
            return powerTable[optimalCoordinateIndex][1];
        }

        else {
            for (int i = 0; i < powerTable.length - 1; i++) {
                if (distance >= powerTable[i][0] && distance <= powerTable[i + 1][0]) {
                    double d0 = powerTable[i][0];
                    double d1 = powerTable[i + 1][0];
                    double p0 = powerTable[i][1];
                    double p1 = powerTable[i + 1][1];

                    // Linear interpolation formula: y = y0 + (x - x0) * ((y1 - y0) / (x1 - x0))
                    return p0 + (distance - d0) * ((p1 - p0) / (d1 - d0));
                }
            }
        }

        return powerTable[0][1]; // default return, should never reach here
    }

    
}

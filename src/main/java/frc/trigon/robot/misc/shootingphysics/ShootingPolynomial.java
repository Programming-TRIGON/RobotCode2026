package frc.trigon.robot.misc.shootingphysics;

public class ShootingPolynomial {
    private final double[] coefficients;

    public ShootingPolynomial(double... coefficients) {
        this.coefficients = coefficients;
    }

    public double evaluate(double distanceFromTargetMeters, double robotVelocityMetersPerSecond) {
        double result = 0.0;
        int col = 0;
        for (int totalDegree = 0; totalDegree <= 3; totalDegree++) {
            for (int xPower = 0; xPower <= totalDegree; xPower++) {
                int vPower = totalDegree - xPower;
                result += coefficients[col] * Math.pow(distanceFromTargetMeters, xPower) * Math.pow(robotVelocityMetersPerSecond, vPower);
                col++;
            }
        }
        return result;
    }
}

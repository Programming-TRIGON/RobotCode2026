package frc.trigon.robot.misc.shootingphysics;

import frc.trigon.lib.utilities.flippable.FlippableTranslation2d;

public class ShootingConstants {
    public static final ShootingPolynomial
            SHOOTING_VELOCITY_MPS_POLYNOMIAL = new ShootingPolynomial(
            4.767958996031e+00,
            -7.728484623310e-02,
            4.251929181565e-01,
            1.040608899068e-01,
            -1.983523275604e-01,
            7.486351577742e-02,
            -4.191789774152e-04,
            -9.365805191577e-03,
            1.565699698698e-02,
            -5.277858971999e-03
    ),
            HOOD_ANGLE_RADIANS_POLYNOMIAL = new ShootingPolynomial(
                    1.623183036320e+00,
                    2.955241202250e-01,
                    -3.475512327334e-01,
                    3.482802254703e-02,
                    -8.015586879455e-02,
                    5.575569393422e-02,
                    9.848403107275e-04,
                    -3.802057779430e-03,
                    6.113602843587e-03,
                    -3.030190410998e-03
            );

    public static final FlippableTranslation2d HUB_POSITION = new FlippableTranslation2d(4.636, 4, true); // TODO: Update to actual hub position
}

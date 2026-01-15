package frc.trigon.robot.misc.shootingphysics;

import frc.trigon.lib.utilities.flippable.FlippableTranslation2d;

public class ShootingConstants {
    public static final ShootingPolynomial
            SHOOTING_VELOCITY_MPS_POLYNOMIAL = new ShootingPolynomial(
            5.146747052198e+00,
            -1.643594256333e-02,
            3.450899319299e-01,
            9.507680110304e-02,
            -1.874843832199e-01,
            8.292659454650e-02,
            -2.198145568214e-03,
            -5.599197896374e-03,
            1.447610537365e-02,
            -5.717921526029e-03
    ),
            HOOD_ANGLE_RADIANS_POLYNOMIAL = new ShootingPolynomial(
                    1.607273351413e+00,
                    2.571395162932e-01,
                    -2.879842443021e-01,
                    2.020244109890e-02,
                    -5.508424076508e-02,
                    4.455905420498e-02,
                    -6.923840792824e-04,
                    -1.315012045184e-03,
                    4.085003609577e-03,
                    -2.473243914566e-03
            );

    public static final FlippableTranslation2d HUB_POSITION = new FlippableTranslation2d(4.636, 4, true); // TODO: Update to actual hub position
}

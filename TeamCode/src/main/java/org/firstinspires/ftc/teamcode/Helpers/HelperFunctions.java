package org.firstinspires.ftc.teamcode.Helpers;
import org.opencv.core.Scalar;

public class HelperFunctions {
    public static double compareScalars(Scalar scal1, Scalar scal2) {
        Scalar diffs = new Scalar(0.0, 0.0, 0.0);
        double sum = 0.0;

        for (int i = 0; i<=2; i++) {
            diffs.val[i] = Math.abs(scal1.val[i]-scal2.val[i])/255;
            sum += diffs.val[i];
        }

        return sum;
    }

    public static double averageScalar() {

    }

}

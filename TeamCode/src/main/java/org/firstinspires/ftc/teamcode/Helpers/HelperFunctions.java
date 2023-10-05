package org.firstinspires.ftc.teamcode.Helpers;
import org.opencv.core.Scalar;

public class HelperFunctions {
    public static double compareScalars(Scalar scal1, Scalar scal2) {
        Scalar diffs = new Scalar(0.0, 0.0, 0.0);
        double sum = 0.0;

        for (int i = 0; i<=2; i++) {
            diffs.val[i] = Math.abs(scal1.val[i]-scal2.val[i]);
            sum += Math.pow(diffs.val[i], 2);
        }

        return Math.sqrt(sum);
    }

    //public static double averageScalar() {

    //}

}

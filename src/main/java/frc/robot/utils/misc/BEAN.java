package frc.robot.utils.misc;

public class BEAN {
    public final double beans;

    public BEAN(double amountOfBeans) {
        this.beans = amountOfBeans;
    }

    public BEAN add(BEAN bean) {
        return new BEAN(beans + bean.beans);
    }

    public BEAN sub(BEAN bean) {
        return new BEAN(beans - bean.beans);
    }

    public static class Beans {
        public static final BEAN STRING_BEAN = new BEAN(0.05);
        public static final BEAN LIMA = new BEAN(0.1);
        public static final BEAN PINTO = new BEAN(0.5);
        public static final BEAN FRENCH_CUT = new BEAN(0.6);
        public static final BEAN BAKED_BEAN = new BEAN(1.0);
    }

}

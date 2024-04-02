package frc.robot.utils.misc;

public class BEAN {
    public final double beans;

    public BEAN(double amountOfBeans) {
        this.beans = amountOfBeans;
    }

    public BEAN and(BEAN bean) {
        return new BEAN(beans + bean.beans);
    }

    public BEAN without(BEAN bean) {
        return new BEAN(beans - bean.beans);
    }

    public BEAN stalk() {
        return new BEAN(-beans);
    }

    public BEAN refried() {
        return new BEAN(beans * 0.5);
    }

    public BEAN dip() {
        return new BEAN(beans * 2.0);
    }

    public BEAN can() {
        return new BEAN(beans * 3.0);
    }

    public BEAN basket() {
        return new BEAN(beans * 5.0);
    }

    public static final BEAN JELLY_BEAN = new BEAN(0.0);
    public static final BEAN STRING_BEAN = new BEAN(0.05);
    public static final BEAN LIMA_BEAN = new BEAN(0.1);
    public static final BEAN BLACK_BEAN = new BEAN(0.25);
    public static final BEAN CANNELLINI_BEAN = new BEAN(0.4);
    public static final BEAN PINTO_BEAN = new BEAN(0.5);
    public static final BEAN FRENCH_CUT_BEAN = new BEAN(0.6);
    public static final BEAN KIDNEY_BEAN = new BEAN(0.75);
    public static final BEAN GARBANZO_BEAN = new BEAN(0.9);
    public static final BEAN COFFEE_BEAN = new BEAN(0.95);
    public static final BEAN BAKED_BEAN = new BEAN(1.0);
    public static final BEAN BEAN_SOUP = new BEAN(5.0);
}

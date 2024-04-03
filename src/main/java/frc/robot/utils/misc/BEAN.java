package frc.robot.utils.misc;

public class BEAN {
    public final double beans;

    public BEAN(double amountOfBeans) {
        this.beans = amountOfBeans;
    }

    /**
     * @param bean - The bean to add to the recipe
     * @return A new bean with the combination of the two (addition)
     */
    public BEAN and(BEAN bean) {
        return new BEAN(beans + bean.beans);
    }

    /**
     * This is the same as and, but we need both for grammatical correctness
     * @param bean - The bean to add to the recipe
     * @return A new bean with the combination of the two (addition)
     */
    public BEAN with(BEAN bean) {
        return new BEAN(beans + bean.beans);
    }

    /**
     * 
     * @param bean - The bean that we are allergic to
     * @return A new bean without the passed in bean (subtraction)
     */
    public BEAN without(BEAN bean) {
        return new BEAN(beans - bean.beans);
    }

    /**
     * This is the same as and, but we need both for grammatical correctness
     * This should only be used after without, but we have no way of enforcing this
     * @param bean - The bean that we are allergic to
     * @return A new bean without the passed in bean (subtraction)
     */
    public BEAN or(BEAN bean) {
        return new BEAN(beans - bean.beans);
    }

    /**
     * @return A new bean with the same power as this bean, but in reverse
     */
    public BEAN stalk() {
        return new BEAN(-beans);
    }

    /**
     * @return A new bean with one fifth the power of this bean
     */
    public BEAN deepfried() {
        return new BEAN(beans * 0.2);
    }

    /**
     * @return A new bean with one half the power of this bean
     */
    public BEAN refried() {
        return new BEAN(beans * 0.5);
    }

    /**
     * @return A new bean with twice the power of this bean
     */
    public BEAN dip() {
        return new BEAN(beans * 2.0);
    }

    /**
     * @return A new bean with 3 times the power of this bean
     */
    public BEAN can() {
        return new BEAN(beans * 3.0);
    }

    /**
     * @return A new bean with 5 times the power of this bean
     */
    public BEAN basket() {
        return new BEAN(beans * 5.0);
    }

    /** 0% */
    public static final BEAN JELLY_BEAN = new BEAN(0.0);
    /** 5% */
    public static final BEAN STRING_BEAN = new BEAN(0.05);
    /** 10% */
    public static final BEAN LIMA_BEAN = new BEAN(0.1);
    /** 25% */
    public static final BEAN BLACK_BEAN = new BEAN(0.25);
    /** 40% */
    public static final BEAN CANNELLINI_BEAN = new BEAN(0.4);
    /** 50% */
    public static final BEAN PINTO_BEAN = new BEAN(0.5);
    /** 60% */
    public static final BEAN FRENCH_CUT_BEAN = new BEAN(0.6);
    /** 75% */
    public static final BEAN KIDNEY_BEAN = new BEAN(0.75);
    /** 90% */
    public static final BEAN GARBANZO_BEAN = new BEAN(0.9);
    /** 95% */
    public static final BEAN COFFEE_BEAN = new BEAN(0.95);
    /** 100% */
    public static final BEAN BAKED_BEAN = new BEAN(1.0);
    /** 500% */
    public static final BEAN BEAN_SOUP = new BEAN(5.0);

    /** IDK, but between 0 and 100% */
    public static final BEAN BEANBOOZLE = new BEAN(Math.random());
}

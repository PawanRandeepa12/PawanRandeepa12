package model;

public class FragilePackage extends Package {
    private double handlingFee;

    public FragilePackage(String id, double weight,
                          String status, double handlingFee) {
        super(id, weight, status);
        this.handlingFee = handlingFee;
    }

    public double getHandlingFee() { return handlingFee; }

    @Override
    public double calculateCost() { return (getWeight() * 150) + handlingFee; }

    @Override
    public String getPackageType() { return "FRAGILE"; }
}

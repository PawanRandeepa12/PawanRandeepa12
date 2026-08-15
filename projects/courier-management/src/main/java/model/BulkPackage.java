package model;

public class BulkPackage extends Package {
    private double discount;

    public BulkPackage(String id, double weight,
                       String status, double discount) {
        super(id, weight, status);
        this.discount = discount;
    }

    public double getDiscount() { return discount; }

    @Override
    public double calculateCost() { return (getWeight() * 150) * (1 - discount / 100); }

    @Override
    public String getPackageType() { return "BULK"; }
}

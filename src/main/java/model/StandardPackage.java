package model;

public class StandardPackage extends Package {
    private int deliveryDays;

    public StandardPackage(String id, double weight,
                           String status, int deliveryDays) {
        super(id, weight, status);
        this.deliveryDays = deliveryDays;
    }

    public int getDeliveryDays() { return deliveryDays; }

    @Override
    public double calculateCost() { return getWeight() * 150; }

    @Override
    public String getPackageType() { return "STANDARD"; }
}

package model;

public abstract class Package {
    private String packageId;
    private double weight;
    private String status;

    public Package(String packageId, double weight, String status) {
        this.packageId = packageId;
        this.weight    = weight;
        this.status    = status;
    }

    public String getPackageId() { return packageId; }
    public double getWeight()    { return weight; }
    public String getStatus()    { return status; }

    public void setStatus(String status) { this.status = status; }

    public abstract double calculateCost();
    public abstract String getPackageType();
}

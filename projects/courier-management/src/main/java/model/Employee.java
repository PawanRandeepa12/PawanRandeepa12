package model;

public abstract class Employee extends Person {
    private String employeeId;
    private double salary;

    public Employee(String employeeId, String name,
                    String phone, double salary) {
        super(employeeId, name, phone);
        this.employeeId = employeeId;
        this.salary     = salary;
    }

    public String getEmployeeId() { return employeeId; }
    public double getSalary()     { return salary; }
    public void setSalary(double salary) { this.salary = salary; }
}

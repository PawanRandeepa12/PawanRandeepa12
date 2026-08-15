package model;

public class Admin extends Employee {

    public Admin(String id, String name, String phone, double salary) {
        super(id, name, phone, salary);
    }

    @Override
    public String getRole() { return "ADMIN"; }
}

package dao;

import database.DBConnection;
import model.DeliveryAgent;
import model.Admin;
import java.sql.*;
import java.util.ArrayList;
import java.util.List;

public class EmployeeDAO {
    private Connection con = DBConnection.getConnection();

    public void addEmployee(DeliveryAgent agent) {
        String sql = "INSERT INTO employees VALUES (?,?,?,?,?)";
        try (PreparedStatement ps = con.prepareStatement(sql)) {
            ps.setString(1, agent.getEmployeeId());
            ps.setString(2, agent.getName());
            ps.setString(3, agent.getPhone());
            ps.setDouble(4, agent.getSalary());
            ps.setString(5, "AGENT");
            ps.executeUpdate();
            System.out.println("Agent added.");
        } catch (SQLException e) { e.printStackTrace(); }
    }

    public void addEmployee(Admin admin) {
        String sql = "INSERT INTO employees VALUES (?,?,?,?,?)";
        try (PreparedStatement ps = con.prepareStatement(sql)) {
            ps.setString(1, admin.getEmployeeId());
            ps.setString(2, admin.getName());
            ps.setString(3, admin.getPhone());
            ps.setDouble(4, admin.getSalary());
            ps.setString(5, "ADMIN");
            ps.executeUpdate();
            System.out.println("Admin added.");
        } catch (SQLException e) { e.printStackTrace(); }
    }

    public void addAgentRecord(DeliveryAgent agent) {
        String sql = "INSERT INTO delivery_agents VALUES (?, ?, ?)";
        try (PreparedStatement ps = con.prepareStatement(sql)) {
            ps.setString(1, agent.getEmployeeId());
            ps.setString(2, agent.getVehicleType());
            ps.setString(3, agent.getZone());
            ps.executeUpdate();
        } catch (SQLException e) { e.printStackTrace(); }
    }

    public List<DeliveryAgent> getAllAgents() {
        List<DeliveryAgent> list = new ArrayList<>();
        String sql = "SELECT e.*, d.vehicle_type, d.zone FROM employees e " +
                     "JOIN delivery_agents d ON e.employee_id = d.employee_id " +
                     "WHERE e.role = 'AGENT'";
        try (Statement st = con.createStatement();
             ResultSet rs = st.executeQuery(sql)) {
            while (rs.next()) {
                list.add(new DeliveryAgent(
                    rs.getString("employee_id"),
                    rs.getString("name"),
                    rs.getString("phone"),
                    rs.getDouble("salary"),
                    rs.getString("vehicle_type"),
                    rs.getString("zone")
                ));
            }
        } catch (SQLException e) { e.printStackTrace(); }
        return list;
    }

    public List<Admin> getAllAdmins() {
        List<Admin> list = new ArrayList<>();
        String sql = "SELECT * FROM employees WHERE role = 'ADMIN'";
        try (Statement st = con.createStatement();
             ResultSet rs = st.executeQuery(sql)) {
            while (rs.next()) {
                list.add(new Admin(
                    rs.getString("employee_id"),
                    rs.getString("name"),
                    rs.getString("phone"),
                    rs.getDouble("salary")
                ));
            }
        } catch (SQLException e) { e.printStackTrace(); }
        return list;
    }

    public void deleteEmployee(String id) {
        String sqlAgent = "DELETE FROM delivery_agents WHERE employee_id = ?";
        String sqlEmp = "DELETE FROM employees WHERE employee_id = ?";
        try {
            PreparedStatement ps1 = con.prepareStatement(sqlAgent);
            ps1.setString(1, id);
            ps1.executeUpdate();
            ps1.close();

            PreparedStatement ps2 = con.prepareStatement(sqlEmp);
            ps2.setString(1, id);
            ps2.executeUpdate();
            ps2.close();
        } catch (SQLException e) { e.printStackTrace(); }
    }
}

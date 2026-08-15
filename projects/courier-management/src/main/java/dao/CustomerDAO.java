package dao;

import database.DBConnection;
import model.Customer;
import java.sql.*;
import java.util.ArrayList;
import java.util.List;

public class CustomerDAO {

    private Connection con = DBConnection.getConnection();

    public void addCustomer(Customer c) {
        String sql = "INSERT INTO customers VALUES (?,?,?,?,?)";
        try (PreparedStatement ps = con.prepareStatement(sql)) {
            ps.setString(1, c.getCustomerId());
            ps.setString(2, c.getName());
            ps.setString(3, c.getPhone());
            ps.setString(4, c.getEmail());
            ps.setString(5, c.getAddress());
            ps.executeUpdate();
            System.out.println("Customer added successfully.");
        } catch (SQLException e) { e.printStackTrace(); }
    }

    public Customer getCustomer(String id) {
        String sql = "SELECT * FROM customers WHERE customer_id = ?";
        try (PreparedStatement ps = con.prepareStatement(sql)) {
            ps.setString(1, id);
            ResultSet rs = ps.executeQuery();
            if (rs.next()) {
                return new Customer(
                    rs.getString("customer_id"),
                    rs.getString("name"),
                    rs.getString("phone"),
                    rs.getString("email"),
                    rs.getString("address")
                );
            }
        } catch (SQLException e) { e.printStackTrace(); }
        return null;
    }

    public List<Customer> getAllCustomers() {
        List<Customer> list = new ArrayList<>();
        String sql = "SELECT * FROM customers";
        try (Statement st = con.createStatement();
             ResultSet rs = st.executeQuery(sql)) {
            while (rs.next()) {
                list.add(new Customer(
                    rs.getString("customer_id"),
                    rs.getString("name"),
                    rs.getString("phone"),
                    rs.getString("email"),
                    rs.getString("address")
                ));
            }
        } catch (SQLException e) { e.printStackTrace(); }
        return list;
    }

    public void updateCustomer(Customer c) {
        String sql = "UPDATE customers SET name=?, phone=?, email=?, address=? " +
                     "WHERE customer_id=?";
        try (PreparedStatement ps = con.prepareStatement(sql)) {
            ps.setString(1, c.getName());
            ps.setString(2, c.getPhone());
            ps.setString(3, c.getEmail());
            ps.setString(4, c.getAddress());
            ps.setString(5, c.getCustomerId());
            ps.executeUpdate();
            System.out.println("Customer updated.");
        } catch (SQLException e) { e.printStackTrace(); }
    }

    public void deleteCustomer(String id) {
        String sql = "DELETE FROM customers WHERE customer_id = ?";
        try (PreparedStatement ps = con.prepareStatement(sql)) {
            ps.setString(1, id);
            ps.executeUpdate();
            System.out.println("Customer deleted.");
        } catch (SQLException e) { e.printStackTrace(); }
    }
}

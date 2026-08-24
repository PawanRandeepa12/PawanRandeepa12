package dao;

import database.DBConnection;
import model.Seller;

import java.sql.Connection;
import java.sql.PreparedStatement;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.sql.Statement;
import java.util.ArrayList;
import java.util.List;

/**
 * Data-access layer for sellers (MSSQL — courier_db).
 *
 * Required table (run once):
 * <pre>
 * CREATE TABLE sellers (
 *     seller_id       VARCHAR(20)  PRIMARY KEY,
 *     name            VARCHAR(100) NOT NULL,
 *     phone           VARCHAR(20),
 *     shop_name       VARCHAR(100),
 *     email           VARCHAR(100),
 *     pickup_address  VARCHAR(200),
 *     commission_rate FLOAT        DEFAULT 0,
 *     orders_handled  INT          DEFAULT 0,
 *     active          BIT          DEFAULT 1
 * );
 * </pre>
 */
public class SellerDAO {

    /** Lazy connection so the UI still opens when the DB is unreachable. */
    private Connection connection() {
        return DBConnection.getConnection();
    }

    private Seller mapRow(ResultSet rs) throws SQLException {
        return new Seller(
                rs.getString("seller_id"),
                rs.getString("name"),
                rs.getString("phone"),
                rs.getString("shop_name"),
                rs.getString("email"),
                rs.getString("pickup_address"),
                rs.getDouble("commission_rate"),
                rs.getInt("orders_handled"),
                rs.getBoolean("active")
        );
    }

    public boolean addSeller(Seller s) {
        String sql = "INSERT INTO sellers VALUES (?,?,?,?,?,?,?,?,?)";
        try (PreparedStatement ps = connection().prepareStatement(sql)) {
            ps.setString(1, s.getSellerId());
            ps.setString(2, s.getName());
            ps.setString(3, s.getPhone());
            ps.setString(4, s.getShopName());
            ps.setString(5, s.getEmail());
            ps.setString(6, s.getPickupAddress());
            ps.setDouble(7, s.getCommissionRate());
            ps.setInt(8, s.getOrdersHandled());
            ps.setBoolean(9, s.isActive());
            ps.executeUpdate();
            System.out.println("Seller added successfully.");
            return true;
        } catch (Exception e) {
            System.out.println("Add seller failed: " + e.getMessage());
            return false;
        }
    }

    public Seller getSeller(String id) {
        String sql = "SELECT * FROM sellers WHERE seller_id = ?";
        try (PreparedStatement ps = connection().prepareStatement(sql)) {
            ps.setString(1, id);
            try (ResultSet rs = ps.executeQuery()) {
                if (rs.next()) {
                    return mapRow(rs);
                }
            }
        } catch (Exception e) {
            System.out.println("Get seller failed: " + e.getMessage());
        }
        return null;
    }

    public List<Seller> getAllSellers() {
        List<Seller> list = new ArrayList<>();
        String sql = "SELECT * FROM sellers";
        try (Statement st = connection().createStatement();
             ResultSet rs = st.executeQuery(sql)) {
            while (rs.next()) {
                list.add(mapRow(rs));
            }
        } catch (Exception e) {
            System.out.println("Load sellers failed: " + e.getMessage());
        }
        return list;
    }

    /** Updates everything except seller_id and orders_handled (business counter). */
    public boolean updateSeller(Seller s) {
        String sql = "UPDATE sellers SET name=?, phone=?, shop_name=?, email=?, " +
                     "pickup_address=?, commission_rate=?, active=? WHERE seller_id=?";
        try (PreparedStatement ps = connection().prepareStatement(sql)) {
            ps.setString(1, s.getName());
            ps.setString(2, s.getPhone());
            ps.setString(3, s.getShopName());
            ps.setString(4, s.getEmail());
            ps.setString(5, s.getPickupAddress());
            ps.setDouble(6, s.getCommissionRate());
            ps.setBoolean(7, s.isActive());
            ps.setString(8, s.getSellerId());
            ps.executeUpdate();
            System.out.println("Seller updated.");
            return true;
        } catch (Exception e) {
            System.out.println("Update seller failed: " + e.getMessage());
            return false;
        }
    }

    public boolean deleteSeller(String id) {
        String sql = "DELETE FROM sellers WHERE seller_id = ?";
        try (PreparedStatement ps = connection().prepareStatement(sql)) {
            ps.setString(1, id);
            ps.executeUpdate();
            System.out.println("Seller deleted.");
            return true;
        } catch (Exception e) {
            System.out.println("Delete seller failed: " + e.getMessage());
            return false;
        }
    }
}

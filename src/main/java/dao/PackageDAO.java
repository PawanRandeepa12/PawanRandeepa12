package dao;

import database.DBConnection;
import model.*;
import java.sql.*;

public class PackageDAO {

    private Connection con = DBConnection.getConnection();

    public void addPackage(Package p) {
        String sql = "INSERT INTO packages VALUES (?,?,?,?,?,?,?,?)";
        try (PreparedStatement ps = con.prepareStatement(sql)) {
            ps.setString(1, p.getPackageId());
            ps.setDouble(2, p.getWeight());
            ps.setString(3, p.getStatus());
            ps.setString(4, p.getPackageType());
            ps.setDouble(5, p.calculateCost());

            if (p instanceof StandardPackage sp) {
                ps.setInt   (6, sp.getDeliveryDays());
                ps.setDouble(7, 0);
                ps.setDouble(8, 0);
            } else if (p instanceof FragilePackage fp) {
                ps.setInt   (6, 0);
                ps.setDouble(7, fp.getHandlingFee());
                ps.setDouble(8, 0);
            } else if (p instanceof BulkPackage bp) {
                ps.setInt   (6, 0);
                ps.setDouble(7, 0);
                ps.setDouble(8, bp.getDiscount());
            }

            ps.executeUpdate();
            System.out.println("Package added.");
        } catch (SQLException e) { e.printStackTrace(); }
    }

    public void updateStatus(String packageId, String status) {
        String sql = "UPDATE packages SET status=? WHERE package_id=?";
        try (PreparedStatement ps = con.prepareStatement(sql)) {
            ps.setString(1, status);
            ps.setString(2, packageId);
            ps.executeUpdate();
        } catch (SQLException e) { e.printStackTrace(); }
    }
}

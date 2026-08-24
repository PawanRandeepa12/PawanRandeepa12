package gui;

import dao.SellerDAO;
import model.Seller;

import javax.swing.*;
import javax.swing.table.DefaultTableModel;
import java.awt.*;
import java.util.List;

/**
 * Seller module UI — manages registered sellers (merchants who hand over
 * parcels to the courier company). Inherits the shared window chrome and
 * styling helpers from CoreUI, so this class only contains seller logic.
 *
 * Module owner: Pawan Jayarathna
 */
public class SellerUI extends CoreUI {

    private final SellerDAO sellerDAO = new SellerDAO();

    // ---- form fields ----
    private final JTextField idField         = new JTextField();
    private final JTextField nameField       = new JTextField();
    private final JTextField phoneField      = new JTextField();
    private final JTextField shopField       = new JTextField();
    private final JTextField emailField      = new JTextField();
    private final JTextField addressField    = new JTextField();
    private final JTextField commissionField = new JTextField("5.0");
    private final JCheckBox  activeCheck     = new JCheckBox("Active seller", true);

    // ---- sellers table ----
    private final DefaultTableModel tableModel = new DefaultTableModel(
            new Object[]{"ID", "Name", "Phone", "Shop", "Email",
                         "Pickup Address", "Commission %", "Orders", "Active"}, 0) {
        @Override
        public boolean isCellEditable(int row, int column) { return false; }
    };
    private final JTable sellerTable = makeTable(tableModel);

    // ---- KPI cards ----
    private final JPanel totalCard   = makeInfoCard("Registered Sellers", "0", new Color(0, 123, 255));
    private final JPanel activeCard  = makeInfoCard("Active Sellers",     "0", new Color(40, 167, 69));
    private final JPanel averageCard = makeInfoCard("Avg Commission %",   "0", new Color(255, 193, 7));

    public SellerUI() {
        super("Seller Management", "Module: Seller  |  DB: courier_db (MSSQL)");
        activeCheck.setFont(FONT_BODY);
        activeCheck.setForeground(TEXT_DARK);
        setContent(buildContent());
        bindRowSelection();
        refreshTable();
    }

    // ================== LAYOUT ==================
    private JComponent buildContent() {
        JPanel root = contentPanel(new BorderLayout(10, 10));

        // --- top block: KPI cards, seller form, action buttons ---
        JPanel top = new JPanel(new BorderLayout(10, 10));
        top.setOpaque(false);

        JPanel cards = new JPanel(new GridLayout(1, 3, 12, 0));
        cards.setOpaque(false);
        cards.setPreferredSize(new Dimension(0, 92));
        cards.add(totalCard);
        cards.add(activeCard);
        cards.add(averageCard);
        top.add(cards, BorderLayout.NORTH);

        JPanel form = new JPanel(new GridLayout(4, 4, 10, 6));
        form.setBackground(Color.WHITE);
        form.setBorder(BorderFactory.createTitledBorder("Seller Details"));
        addFormField(form, "Seller ID *",          idField);
        addFormField(form, "Full Name *",          nameField);
        addFormField(form, "Phone",                phoneField);
        addFormField(form, "Shop / Business Name", shopField);
        addFormField(form, "Email",                emailField);
        addFormField(form, "Pickup Address",       addressField);
        addFormField(form, "Commission % (5.0)",   commissionField);
        form.add(new JLabel());            // spacer keeps the grid aligned
        form.add(activeCheck);
        top.add(form, BorderLayout.CENTER);

        JPanel actions = new JPanel(new FlowLayout(FlowLayout.LEFT, 10, 6));
        actions.setOpaque(false);
        actions.add(styledButton("Add Seller",          this::addSeller));
        actions.add(styledButton("Update Seller",       this::updateSeller));
        actions.add(styledButton("Delete Seller",       this::deleteSeller));
        actions.add(styledButton("Calculate Commission", this::calculateCommission));
        actions.add(styledButton("Clear Form",          this::clearForm));
        top.add(actions, BorderLayout.SOUTH);

        root.add(top, BorderLayout.NORTH);

        // --- bottom block: sellers table ---
        root.add(section("Registered Sellers", new JScrollPane(sellerTable)), BorderLayout.CENTER);

        return root;
    }

    // ================== ACTIONS ==================
    private void addSeller() {
        Seller seller = readForm();
        if (seller == null) return;
        if (!sellerDAO.addSeller(seller)) {
            showError("Could not save the seller.\nCheck the database connection and that the ID is not a duplicate.");
            return;
        }
        refreshTable();
        clearForm();
        setStatus("Seller " + seller.getSellerId() + " added.");
        showInfo("Seller added successfully.");
    }

    private void updateSeller() {
        Seller seller = readForm();
        if (seller == null) return;
        if (!sellerDAO.updateSeller(seller)) {
            showError("Could not update seller " + seller.getSellerId() + ".");
            return;
        }
        refreshTable();
        setStatus("Seller " + seller.getSellerId() + " updated.");
        showInfo("Seller updated successfully.");
    }

    private void deleteSeller() {
        String id = idField.getText().trim();
        if (id.isEmpty()) {
            showError("Select a seller from the table (or type the Seller ID) first.");
            return;
        }
        int choice = JOptionPane.showConfirmDialog(this,
                "Delete seller " + id + "? This cannot be undone.",
                "Confirm Delete", JOptionPane.YES_NO_OPTION, JOptionPane.WARNING_MESSAGE);
        if (choice != JOptionPane.YES_OPTION) return;
        if (!sellerDAO.deleteSeller(id)) {
            showError("Could not delete seller " + id + ".");
            return;
        }
        refreshTable();
        clearForm();
        setStatus("Seller " + id + " deleted.");
    }

    /** Uses the commission rate in the form to compute the payout for an order. */
    private void calculateCommission() {
        Seller seller = readForm();
        if (seller == null) return;
        String input = JOptionPane.showInputDialog(this,
                "Order amount (Rs.):", "Commission Calculator", JOptionPane.QUESTION_MESSAGE);
        if (input == null) return; // cancelled
        try {
            double amount = Double.parseDouble(input.trim());
            double commission = seller.calculateCommission(amount);
            showInfo(String.format("%s on order Rs. %.2f earns:%nCommission: Rs. %.2f (%.1f%%)",
                    seller.getName(), amount, commission, seller.getCommissionRate()));
        } catch (NumberFormatException ex) {
            showError("Please enter a valid numeric amount.");
        }
    }

    private void clearForm() {
        idField.setText("");
        nameField.setText("");
        phoneField.setText("");
        shopField.setText("");
        emailField.setText("");
        addressField.setText("");
        commissionField.setText("5.0");
        activeCheck.setSelected(true);
        sellerTable.clearSelection();
        setStatus("Form cleared.");
    }

    // ================== HELPERS ==================
    /** Validates the form and builds a Seller object; null + message on error. */
    private Seller readForm() {
        String id   = idField.getText().trim();
        String name = nameField.getText().trim();
        if (id.isEmpty())   { showError("Seller ID is required.");   return null; }
        if (name.isEmpty()) { showError("Seller name is required."); return null; }
        double rate;
        try {
            rate = Double.parseDouble(commissionField.getText().trim());
        } catch (NumberFormatException e) {
            showError("Commission must be numeric, e.g. 5.0 (for 5%).");
            return null;
        }
        return new Seller(id, name, phoneField.getText().trim(),
                shopField.getText().trim(), emailField.getText().trim(),
                addressField.getText().trim(), rate, 0, activeCheck.isSelected());
    }

    private void refreshTable() {
        tableModel.setRowCount(0);
        List<Seller> sellers = sellerDAO.getAllSellers();
        int activeCount = 0;
        double rateSum = 0;
        for (Seller s : sellers) {
            if (s.isActive()) activeCount++;
            rateSum += s.getCommissionRate();
            tableModel.addRow(new Object[]{
                    s.getSellerId(), s.getName(), s.getPhone(), s.getShopName(),
                    s.getEmail(), s.getPickupAddress(), s.getCommissionRate(),
                    s.getOrdersHandled(), s.isActive()});
        }
        updateCard(totalCard,   String.valueOf(sellers.size()));
        updateCard(activeCard,  String.valueOf(activeCount));
        updateCard(averageCard, sellers.isEmpty() ? "0.0"
                : String.format("%.1f", rateSum / sellers.size()));
        setStatus(sellers.size() + " seller(s) loaded.");
    }

    /** Clicking a row loads it back into the form for editing. */
    private void bindRowSelection() {
        sellerTable.getSelectionModel().addListSelectionListener(e -> {
            if (e.getValueIsAdjusting()) return;
            int row = sellerTable.getSelectedRow();
            if (row < 0) return;
            idField.setText(String.valueOf(sellerTable.getValueAt(row, 0)));
            nameField.setText(String.valueOf(sellerTable.getValueAt(row, 1)));
            phoneField.setText(String.valueOf(sellerTable.getValueAt(row, 2)));
            shopField.setText(String.valueOf(sellerTable.getValueAt(row, 3)));
            emailField.setText(String.valueOf(sellerTable.getValueAt(row, 4)));
            addressField.setText(String.valueOf(sellerTable.getValueAt(row, 5)));
            commissionField.setText(String.valueOf(sellerTable.getValueAt(row, 6)));
            activeCheck.setSelected(Boolean.TRUE.equals(sellerTable.getValueAt(row, 8)));
            setStatus("Editing seller " + sellerTable.getValueAt(row, 0) + ".");
        });
    }

    // ================== ENTRY POINT ==================
    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> {
            applySystemLookAndFeel();
            new SellerUI().showWindow();
        });
    }
}

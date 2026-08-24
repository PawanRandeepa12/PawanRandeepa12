package gui;

import javax.swing.*;
import javax.swing.border.EmptyBorder;
import javax.swing.table.DefaultTableModel;
import java.awt.*;

/**
 * Shared base window for every module UI of the Courier Management System.
 * Provides the common application chrome (dark header band + footer status
 * bar) and a small styling toolkit (buttons, info cards, tables, forms,
 * dialogs) so subclasses only build their own screen content.
 *
 * Usage:
 *   class MyModuleUI extends CoreUI {
 *       MyModuleUI() {
 *           super("My Module", "subtitle");
 *           setContent(buildContent());
 *       }
 *   }
 */
public abstract class CoreUI extends JFrame {

    // ---- shared theme ----
    protected static final Color PRIMARY    = new Color(30, 41, 59);
    protected static final Color BG_LIGHT   = new Color(245, 247, 250);
    protected static final Color TEXT_DARK  = new Color(60, 70, 90);
    protected static final Color TEXT_MUTED = new Color(110, 120, 140);

    protected static final Font FONT_TITLE = new Font("Segoe UI", Font.BOLD, 20);
    protected static final Font FONT_SUB   = new Font("Segoe UI", Font.PLAIN, 12);
    protected static final Font FONT_BODY  = new Font("Segoe UI", Font.PLAIN, 13);
    protected static final Font FONT_BOLD  = new Font("Segoe UI", Font.BOLD, 13);

    private final JLabel statusLabel = new JLabel(" Ready");

    protected CoreUI(String moduleTitle, String subtitle) {
        super("Courier Management System");
        setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
        setSize(1040, 720);
        setMinimumSize(new Dimension(900, 600));
        setLocationRelativeTo(null);
        setLayout(new BorderLayout());

        // ---- shared header band ----
        JPanel header = new JPanel(new BorderLayout());
        header.setBackground(PRIMARY);
        header.setPreferredSize(new Dimension(0, 62));

        JLabel title = new JLabel("  " + moduleTitle);
        title.setForeground(Color.WHITE);
        title.setFont(FONT_TITLE);
        header.add(title, BorderLayout.WEST);

        JLabel sub = new JLabel(subtitle + "  ");
        sub.setForeground(new Color(180, 190, 210));
        sub.setFont(FONT_SUB);
        header.add(sub, BorderLayout.EAST);

        add(header, BorderLayout.NORTH);

        // ---- shared footer status bar ----
        JPanel footer = new JPanel(new FlowLayout(FlowLayout.LEFT));
        footer.setBackground(new Color(240, 242, 245));
        footer.setBorder(BorderFactory.createMatteBorder(1, 0, 0, 0, new Color(200, 205, 215)));
        statusLabel.setFont(FONT_SUB);
        statusLabel.setForeground(new Color(80, 90, 110));
        footer.add(statusLabel);
        add(footer, BorderLayout.SOUTH);
    }

    /** Padded, light-background panel used as the body of a screen. */
    protected JPanel contentPanel(LayoutManager layout) {
        JPanel p = new JPanel(layout);
        p.setBackground(BG_LIGHT);
        p.setBorder(new EmptyBorder(12, 12, 12, 12));
        return p;
    }

    /** Places the main screen content between the shared header and footer. */
    protected void setContent(JComponent content) {
        add(content, BorderLayout.CENTER);
    }

    /** Updates the text on the shared footer status bar. */
    protected void setStatus(String text) {
        statusLabel.setText(" " + text);
    }

    /** Standard action button used across all modules. */
    protected JButton styledButton(String text, Runnable action) {
        JButton btn = new JButton(text);
        btn.setBackground(PRIMARY);
        btn.setForeground(Color.WHITE);
        btn.setFocusPainted(false);
        btn.setFont(FONT_BODY);
        btn.setBorder(new EmptyBorder(8, 16, 8, 16));
        btn.setCursor(new Cursor(Cursor.HAND_CURSOR));
        btn.addActionListener(e -> action.run());
        return btn;
    }

    /** Small KPI card (title + big value). Update later via updateCard(). */
    protected JPanel makeInfoCard(String title, String value, Color accent) {
        JPanel card = new JPanel(new BorderLayout());
        card.setBackground(Color.WHITE);
        card.setBorder(BorderFactory.createCompoundBorder(
                BorderFactory.createMatteBorder(0, 4, 0, 0, accent),
                new EmptyBorder(12, 14, 12, 14)));
        JLabel t = new JLabel(title);
        t.setFont(FONT_BOLD);
        t.setForeground(TEXT_DARK);
        JLabel v = new JLabel(value);
        v.setFont(new Font("Segoe UI", Font.BOLD, 24));
        v.setForeground(accent);
        card.add(t, BorderLayout.NORTH);
        card.add(v, BorderLayout.CENTER);
        card.putClientProperty("valueLabel", v);
        return card;
    }

    /** Updates the big value label of a card created with makeInfoCard(). */
    protected void updateCard(JPanel card, String value) {
        Object label = card.getClientProperty("valueLabel");
        if (label instanceof JLabel) {
            ((JLabel) label).setText(value);
        }
    }

    /** Consistently styled, read-only-by-default table. */
    protected JTable makeTable(DefaultTableModel tableModel) {
        JTable table = new JTable(tableModel);
        table.setRowHeight(24);
        table.setFont(FONT_BODY);
        table.setGridColor(new Color(215, 220, 230));
        table.setSelectionBackground(new Color(190, 210, 245));
        table.getTableHeader().setFont(FONT_BOLD);
        table.getTableHeader().setBackground(new Color(235, 238, 244));
        table.getTableHeader().setReorderingAllowed(false);
        return table;
    }

    /** Wraps a component in a white, titled section. */
    protected JPanel section(String title, JComponent inner) {
        JPanel wrap = new JPanel(new BorderLayout());
        wrap.setBackground(Color.WHITE);
        wrap.setBorder(BorderFactory.createTitledBorder(title));
        wrap.add(inner, BorderLayout.CENTER);
        return wrap;
    }

    /** Adds a "Label: [field]" pair to a grid-based form panel. */
    protected void addFormField(JPanel form, String label, JComponent field) {
        JLabel lbl = new JLabel(label);
        lbl.setFont(FONT_BODY);
        lbl.setForeground(TEXT_DARK);
        form.add(lbl);
        form.add(field);
    }

    protected void showInfo(String message) {
        JOptionPane.showMessageDialog(this, message,
                "Courier Management System", JOptionPane.INFORMATION_MESSAGE);
    }

    protected void showError(String message) {
        JOptionPane.showMessageDialog(this, message,
                "Error", JOptionPane.ERROR_MESSAGE);
    }

    /** Applies the native OS look and feel (call once before creating UIs). */
    protected static void applySystemLookAndFeel() {
        try {
            UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
        } catch (Exception ignored) { }
    }

    /** Displays the window centered on screen. */
    public void showWindow() {
        setVisible(true);
    }
}

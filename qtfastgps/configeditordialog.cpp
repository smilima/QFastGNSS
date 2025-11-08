#include "configeditordialog.h"
#include <QTextEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFile>
#include <QTextStream>
#include <QMessageBox>
#include <QTextCharFormat>
#include <QBrush>
#include <QColor>

// ============================================================================
// CaretLineHighlighter Implementation
// ============================================================================

CaretLineHighlighter::CaretLineHighlighter(QTextDocument *parent)
    : QSyntaxHighlighter(parent)
{
    // Configure the format for lines starting with ^
    // You can customize these colors to your preference
    m_caretLineFormat.setBackground(QBrush(QColor(255, 255, 200))); // Light yellow background
    m_caretLineFormat.setForeground(QBrush(QColor(0, 100, 0)));     // Dark green text
    // Optionally make it bold:
    // m_caretLineFormat.setFontWeight(QFont::Bold);
}

void CaretLineHighlighter::highlightBlock(const QString &text)
{
    // Check if the line starts with ^
    if (text.startsWith('^')) {
        // Apply the format to the entire line
        setFormat(0, text.length(), m_caretLineFormat);
    }
}

ConfigEditorDialog::ConfigEditorDialog(QWidget *parent)
    : QDialog(parent)
{
    setWindowTitle("Edit Config File (fastgps_config.txt)");
    resize(600, 400); // Set a reasonable default size

    // 1. Create the main text edit area
    m_textEdit = new QTextEdit(this);
    m_textEdit->setFontFamily("Monospace");

    m_highlighter = new CaretLineHighlighter(m_textEdit->document());

    // 2. Create the buttons
    m_saveButton = new QPushButton("Save and Close", this);
    m_cancelButton = new QPushButton("Cancel", this);

    // 3. Create the button layout
    QHBoxLayout *buttonLayout = new QHBoxLayout;
    buttonLayout->addStretch(); // Pushes buttons to the right
    buttonLayout->addWidget(m_cancelButton);
    buttonLayout->addWidget(m_saveButton);

    // 4. Create the main layout
    QVBoxLayout *mainLayout = new QVBoxLayout(this);
    mainLayout->addWidget(m_textEdit);
    mainLayout->addLayout(buttonLayout);
    setLayout(mainLayout);

    // 5. Connect signals
    connect(m_saveButton, &QPushButton::clicked, this, &ConfigEditorDialog::onSaveAndClose);
    connect(m_cancelButton, &QPushButton::clicked, this, &QDialog::reject); // 'reject' just closes the dialog

    // 6. Load the file content
    loadFile();
}

void ConfigEditorDialog::loadFile()
{
    QFile file("fastgps_config.txt");
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QMessageBox::warning(this, "Error", "Could not open fastgps_config.txt for reading.");
        m_textEdit->setPlainText("Error: Could not load file.");
        m_textEdit->setReadOnly(true);
        m_saveButton->setEnabled(false);
        return;
    }

    QTextStream in(&file);
    m_textEdit->setPlainText(in.readAll());
    file.close();

    m_highlighter->rehighlight();
}

void ConfigEditorDialog::onSaveAndClose()
{
    QFile file("fastgps_config.txt");
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
        QMessageBox::critical(this, "Error", "Could not open fastgps_config.txt for writing.");
        return;
    }

    QTextStream out(&file);
    out << m_textEdit->toPlainText();
    file.close();

    // 'accept' closes the dialog and signals success
    accept();
}

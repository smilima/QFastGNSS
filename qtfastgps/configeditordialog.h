#ifndef CONFIGEDITORDIALOG_H
#define CONFIGEDITORDIALOG_H

#include <QDialog>
#include <QSyntaxHighlighter>
#include <QTextDocument>

// Forward declarations
class QTextEdit;
class QPushButton;

// Syntax highlighter for lines starting with ^
class CaretLineHighlighter : public QSyntaxHighlighter
{
    Q_OBJECT

public:
    explicit CaretLineHighlighter(QTextDocument *parent = nullptr);

protected:
    void highlightBlock(const QString &text) override;

private:
    QTextCharFormat m_caretLineFormat;
};

class ConfigEditorDialog : public QDialog
{
    Q_OBJECT

public:
    // Pass the parent window (MainWindow) to the constructor
    explicit ConfigEditorDialog(QWidget *parent = nullptr);

private slots:
    // Slot to handle saving the file
    void onSaveAndClose();

private:
    // Helper function to load the file contents
    void loadFile();

    QTextEdit *m_textEdit;
    QPushButton *m_saveButton;
    QPushButton *m_cancelButton;
    CaretLineHighlighter *m_highlighter;
};



#endif // CONFIGEDITORDIALOG_H

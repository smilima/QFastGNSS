// fastgps-qt/qtfastgps/mainwindow.h
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QString>
#include <QFuture> // <-- ADD THIS INCLUDE


// Forward declarations for Qt classes
class QPushButton;
class QTextEdit;
class QCloseEvent;
class QAction;
class QMenu;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

signals:
    // Signal to safely send a log message from the worker thread to the GUI thread
    void logMessageReady(const QString &message);

public slots:
    // Slot to append the message to the text box
    void appendLogMessage(const QString &message);

private slots:
    // Slot connected to the "Start" button's click
    void onStartButtonClicked();
    void onStopButtonClicked();
    void onOpenFileEditor();

    // --- ADD THIS PROTECTED SECTION ---
protected:
    // This function is called when the user clicks the "X" button
    void closeEvent(QCloseEvent *event) override;
    // ------------------------------------

private:
    void setupMenuBar();

    QPushButton *startButton;
    QTextEdit *logBox;
    QFuture<int> m_gpsFuture; // <-- ADD THIS MEMBER to store the future
    bool m_isRunning = false;

    QMenu *m_fileMenu;
    QAction *m_openAction;
    QAction *m_exitAction;
};

// Global C-function declaration for the library
// (This is a C++ project, so no extern "C" is needed here)
void fastgps_printf(const char *format, ...);

#endif // MAINWINDOW_H

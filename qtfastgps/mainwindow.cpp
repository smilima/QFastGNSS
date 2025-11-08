// fastgps-qt/qtfastgps/mainwindow.cpp
#include <QApplication>
#include "mainwindow.h"
#include "configeditordialog.h"
#include <QPushButton>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QWidget>
#include <QCloseEvent>
#include <QMenuBar>
#include <QMenu>
#include <QAction>
#include <cstdarg> // For va_list, va_start, va_end

// Includes for string manipulation
#include <string>
#include <algorithm>
#include "fastgps.h" // This now brings in the g_stop_processing declaration

// Global pointer to the main window for the callback
static MainWindow* g_MainWindowInstance = nullptr;

void fastgps_printf(const char *format, ...)
{
    va_list args;
    va_start(args, format);

    va_list args_copy;
    va_copy(args_copy, args);
    int len = vsnprintf(nullptr, 0, format, args_copy);
    va_end(args_copy);

    if (len < 0) {
        va_end(args);
        return;
    }

    std::string msgbuf(len + 1, '\0');
    vsnprintf(&msgbuf[0], len + 1, format, args);
    va_end(args);

    msgbuf.resize(len);

    // Now we can safely remove the '\r' characters
    msgbuf.erase(std::remove(msgbuf.begin(), msgbuf.end(), '\r'), msgbuf.end());

    if (g_MainWindowInstance) {
        // Emit the signal
        emit g_MainWindowInstance->logMessageReady(QString::fromStdString(msgbuf));
    } else {
        // Fallback to console
        printf("%s", msgbuf.c_str());
        fflush(stdout);
    }
}


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    // Set the global instance for the C callback
    g_MainWindowInstance = this;

    // Set up the main window
    setWindowTitle("GNSS Receiver");
    resize(700, 500);

    // Create main widgets
    QWidget *centralWidget = new QWidget(this);
    QVBoxLayout *mainLayout = new QVBoxLayout(centralWidget);

    logBox = new QTextEdit(this);
    logBox->setReadOnly(true);
    logBox->setFontFamily("Monospace");
    logBox->setPlainText("Welcome to the fastgps software receiver!\nClick 'Start' to begin processing...\n");

    startButton = new QPushButton("Start fastgps Receiver", this);

    // Add widgets to the layout
    mainLayout->addWidget(logBox);
    mainLayout->addWidget(startButton);
    setCentralWidget(centralWidget);

    setupMenuBar();

    // --- Signal/Slot Connections ---
    connect(startButton, &QPushButton::clicked, this, &MainWindow::onStartButtonClicked);
    connect(this, &MainWindow::logMessageReady, this, &MainWindow::appendLogMessage, Qt::QueuedConnection);
}

MainWindow::~MainWindow()
{
    g_MainWindowInstance = nullptr; // Clear the global instance
    fastgps_printf("Window closing, signaling threads to stop...\n");
    get_stop_flag().store(true);
}

void MainWindow::onStartButtonClicked()
{
    logBox->clear();
    logBox->append("Starting fastgps processing in a background thread...");

    // --- UPDATE BUTTON STATE ---
    m_isRunning = true;
    startButton->setText("Stop");

    // --- RE-WIRE BUTTON SIGNAL ---
    disconnect(startButton, &QPushButton::clicked, this, &MainWindow::onStartButtonClicked);
    connect(startButton, &QPushButton::clicked, this, &MainWindow::onStopButtonClicked);

    // Reset the stop flag for the new run
    get_stop_flag().store(false);

    // --- MODIFY THESE LINES ---
    // Store the future in our member variable instead of a local one
    m_gpsFuture = QtConcurrent::run([=](){

        // This code runs in the background
        int result = 0;
        try {
            result = run_fastgps();
        } catch (const std::exception &e) {
            emit logMessageReady(QString("An exception occurred: %1").arg(e.what()));
        } catch (...) {
            emit logMessageReady(QString("An unknown error occurred in fastgps thread."));
        }
        return result;
    });

    // Use a QFutureWatcher to get a signal when the thread is done
    QFutureWatcher<int> *watcher = new QFutureWatcher<int>(this);
    connect(watcher, &QFutureWatcher<int>::finished, [=]() {
        int result = watcher->result();
        appendLogMessage(QString("\n--- Processing finished with code %1 ---").arg(result));
        startButton->setEnabled(true);
        startButton->setText("Start fastgps Receiver");
        watcher->deleteLater();
    });

    watcher->setFuture(m_gpsFuture); // Point the watcher to the member future
}

void MainWindow::onStopButtonClicked()
{
    appendLogMessage("\n--- STOP button clicked: Signaling worker thread to stop... ---");

    // 1. Signal the thread to stop
    get_stop_flag().store(true);

    // 2. Disable the button to prevent multiple clicks
    //    The 'finished' lambda will re-enable it.
    startButton->setEnabled(false);
    startButton->setText("Stopping.  Please wait, this may take a few seonds...");
}

void MainWindow::onOpenFileEditor()
{
    // Stop the receiver if it's running, as the config might change
    if (m_isRunning) {
        appendLogMessage("\n--- Opening config editor. Stopping receiver first... ---");
        onStopButtonClicked();

        // Wait for it to *actually* stop before opening the editor
        m_gpsFuture.waitForFinished();
    }

    // Create and show the modal dialog
    ConfigEditorDialog editorDialog(this);
    editorDialog.exec(); // This blocks until the user clicks "Save" or "Cancel"

    appendLogMessage("\nConfig editor closed.\n");
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    // Check if the background thread is currently running
    if (m_isRunning)
    {
        appendLogMessage("\n--- EXIT REQUESTED: Signaling worker thread to stop... ---\n");

        // 1. Signal the thread to stop
        get_stop_flag().store(true);

        // 2. Wait for the thread to *actually* finish.
        //    This will block the GUI thread, but since the window is
        //    closing, this is acceptable.
        m_gpsFuture.waitForFinished();

        appendLogMessage("--- Worker thread finished. Exiting. ---");
    }

    // 3. Now, accept the event and allow the window to close,
    //    which will terminate the application.
    event->accept();
}

void MainWindow::setupMenuBar()
{
    // 1. Create Actions
    m_openAction = new QAction("&Open Config...", this);
    m_exitAction = new QAction("E&xit", this);

    // 2. Create File Menu and add actions
    m_fileMenu = menuBar()->addMenu("&File");
    m_fileMenu->addAction(m_openAction);
    m_fileMenu->addSeparator();
    m_fileMenu->addAction(m_exitAction);

    // 3. Connect actions to slots
    connect(m_openAction, &QAction::triggered, this, &MainWindow::onOpenFileEditor);
    // Connect Exit to the window's main close slot, which triggers our closeEvent
    connect(m_exitAction, &QAction::triggered, this, &MainWindow::close);
}

// This slot runs in the *main GUI thread*
void MainWindow::appendLogMessage(const QString &message)
{
    // This function should be clean now, as the string is
    // purified in fastgps_printf before it even gets here.
    logBox->moveCursor(QTextCursor::End);
    logBox->insertPlainText(message);
    logBox->ensureCursorVisible();
}

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <cmath>  // For M_PI

// Constructor: runs when MainWindow is created
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    // Setup UI from .ui file
    ui->setupUi(this);

    // Set slider range from 0 to 2π (0 to 200 steps, each step = π/100)
    ui->horizontalSlider->setMinimum(0);
    ui->horizontalSlider->setMaximum(200);
    ui->horizontalSlider->setValue(100);  // Start at π (halfway)

    // Connect slider value changes to update the plotWidget's xmax
    connect(ui->horizontalSlider, &QSlider::valueChanged, this, [=](int value) {
        // Convert slider int to floating point in range [0, 2π]
        double xmax = value * M_PI / 100.0;
        ui->plotWidget->setXMax(xmax);
    });
}

// Destructor: cleans up the UI object when MainWindow is closed
MainWindow::~MainWindow()
{
    delete ui;
}

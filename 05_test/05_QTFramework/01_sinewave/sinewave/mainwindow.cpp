#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "plotwidget.h"  // Include our custom plotting widget

#include <cmath>         // For sin() and M_PI
#include <vector>        // To hold our sine wave data

using namespace std;

// Constructor for the main application window
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    // Load the UI from the Designer-generated file
    ui->setupUi(this); // Loads the GUI layout from mainwindow.ui

    // Create a vector to simulate input data points
    vector<QPointF> sineData;
    vector<QPointF> cosData;
    const int totalPoints = 500;  // Total number of samples

    /*
     * Wave form generator
     */
    // Generate points along the sine wave
    for (int i = 0; i < totalPoints; ++i) {
        double x = static_cast<double>(i) / (totalPoints - 1);  // Normalized X: [0, 1]
        double y = sin(x * 2 * M_PI);  // Y value: sin(2πx)
        sineData.emplace_back(x, y);  // Store the point
    }
    // Generate points along the cosine wave
    for (int i = 0; i < totalPoints; ++i) {
        double x = static_cast<double>(i) / (totalPoints - 1);  // Normalized X: [0, 1]
        double y = cos(x * 2 * M_PI);  // Y value: cos(2πx)
        cosData.emplace_back(x, y);  // Store the point
    }

    // Send the full data array to the plot widget
    ui->plotWidget_1->setData(sineData);
    ui->plotWidget_2->setData(cosData);

    // Configure the slider:
    // - Minimum = 0 points (empty)
    // - Maximum = all points
    // - Start at full view
    ui->horizontalSlider->setMinimum(0);
    ui->horizontalSlider->setMaximum(static_cast<int>(sineData.size()));
    ui->horizontalSlider->setValue(static_cast<int>(sineData.size()));

    // Connect slider value changes to control how many points are visible
    connect(ui->horizontalSlider, &QSlider::valueChanged, this, [=](int value) {
        // This method limits how much of the sine data is visualized
        ui->plotWidget_1->setVisibleCount(value);
        // This method limits how much of the cosine data is visualized
        ui->plotWidget_2->setVisibleCount(value);
    });
}

// Destructor: clean up UI resources
MainWindow::~MainWindow()
{
    delete ui;
}

#include "plotwidget.h"
#include <QPainter>  // For drawing
#include <cmath>     // For math functions like sin()

using namespace std;

// Constructor: simply call the base class constructor
PlotWidget::PlotWidget(QWidget *parent) : QWidget(parent) {}

// Sets the new maximum x-value for the sine wave and triggers a repaint
void PlotWidget::setXMax(double xmax) {
    m_xmax = xmax;
    update();  // This tells Qt to repaint the widget by calling paintEvent()
}

// This function is automatically called by Qt whenever the widget needs to be redrawn
void PlotWidget::paintEvent(QPaintEvent *) {
    QPainter painter(this);  // Create a painter object for drawing on the widget

    // Enable smooth curves (anti-aliasing)
    painter.setRenderHint(QPainter::Antialiasing);

    /*
     * Draw a horizontal axis line in the center:
     *
     * Width of the widget (in pixels)
     * Height of the widget (in pixels)
     */
    painter.drawLine(0, height() / 2, width(), height() / 2);

    // Set pen color and thickness for the sine wave line
    QPen pen(Qt::blue, 2);
    painter.setPen(pen);

    // Variable to keep track of the previous point so we can draw lines between points
    QPoint prev;

    // Loop over each pixel horizontally
    for (int i = 0; i <  width(); ++i) {
        // Map pixel `i` to a value of x between 0 and m_xmax
        double x = m_xmax * i / width();

        // Compute the sine of x
        double y = sin(x);  // y is between -1 and 1

        // Map y from [-1, 1] to pixel height:
        // - When y = 1 → y_pixel = 0 (top of widget)
        // - When y = -1 → y_pixel = h (bottom of widget)
        int y_pixel = static_cast<int>((1 - y) * height() / 2);

        // Create the point in pixel coordinates
        QPoint point(i, y_pixel);

        // After the first point, draw a line from the previous to the current
        if (i > 0)
            painter.drawLine(prev, point);

        // Update prev to current for next iteration
        prev = point;
    }
}

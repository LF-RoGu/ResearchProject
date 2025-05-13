#include "plotwidget.h"
#include <QPainter>  // For custom drawing
#include <algorithm> // For std::min

// Constructor: calls the base QWidget constructor
PlotWidget::PlotWidget(QWidget *parent) : QWidget(parent) {}

// This method sets the full dataset (e.g., all sine wave points)
void PlotWidget::setData(const std::vector<QPointF>& points) {
    m_data = points;                                    // Store the full dataset
    m_visibleCount = static_cast<int>(points.size());   // Show all points by default
    update();                                           // Trigger repaint of the widget
}

// This method limits how many data points are currently shown
void PlotWidget::setVisibleCount(int count) {
    // Make sure we don't exceed the size of the dataset
    m_visibleCount = std::min(count, static_cast<int>(m_data.size()));
    update();  // Repaint the widget with the new subset
}

// This function is called automatically by Qt whenever the widget needs to be redrawn
void PlotWidget::paintEvent(QPaintEvent *) {
    QPainter painter(this);                         // Create painter object
    painter.setRenderHint(QPainter::Antialiasing);  // Smooth line rendering

    int w = width();   // Width of the widget in pixels
    int h = height();  // Height of the widget in pixels

    // Draw a horizontal axis across the middle of the widget
    painter.drawLine(0, h / 2, w, h / 2);

    // Set pen color and line thickness for the sine curve
    QPen pen(Qt::blue, 2);
    painter.setPen(pen);

    // Return early if no data or not enough points to draw a line
    if (m_data.empty() || m_visibleCount < 2)
        return;

    QPoint prev;

    // Iterate over the number of points currently visible
    for (int i = 0; i < m_visibleCount; ++i) {
        // Get normalized x and real y values from data
        double x_norm = m_data[i].x();  // Expecting range [0, 1]
        double y_val  = m_data[i].y();  // Expecting range [-1, 1]

        // Map normalized x to actual pixel width
        int x_pix = static_cast<int>(x_norm * w);

        // Map y to pixel height:
        // y = 1 → top (0 px), y = -1 → bottom (h px)
        int y_pix = static_cast<int>((1 - y_val) * h / 2);

        // Create point in pixel space
        QPoint pt(x_pix, y_pix);

        // Draw a line from the previous point to the current one
        if (i > 0)
            painter.drawLine(prev, pt);

        // Save current point for next iteration
        prev = pt;
    }
}

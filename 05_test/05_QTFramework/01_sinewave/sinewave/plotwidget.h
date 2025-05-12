#ifndef PLOTWIDGET_H
#define PLOTWIDGET_H

/* Base class for all UI elements in Qt */
#include <QWidget>

// PlotWidget is a custom QWidget that draws a sine wave
class PlotWidget : public QWidget
{
    Q_OBJECT // Qt macro that enables signals/slots and meta-object features
public:
    /* Constructor */
    explicit PlotWidget(QWidget *parent = nullptr);
    /* Public method to set the maximum value of x (the slider will change this) */
    void setXMax(double xmax);
protected:
    /* Overridden paintEvent to handle custom drawing logic */
    void paintEvent(QPaintEvent *event) override;

private:
    /* Maximum value of x to draw the sine wave up to (e.g., 2π) */
    double m_xmax = 2 * M_PI;

signals:
};

#endif // PLOTWIDGET_H

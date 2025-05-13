#ifndef PLOTWIDGET_H
#define PLOTWIDGET_H

/* Base class for all UI elements in Qt */
#include <QWidget>

using namespace std;

// PlotWidget is a custom QWidget that draws a sine wave
class PlotWidget : public QWidget
{
    Q_OBJECT // Qt macro that enables signals/slots and meta-object features
public:
    /* Constructor */
    explicit PlotWidget(QWidget *parent = nullptr);
    /* */
    void setData(const vector<QPointF>& points);
    /* */
    void setVisibleCount(int count);
protected:
    /* Overridden paintEvent to handle custom drawing logic */
    void paintEvent(QPaintEvent *event) override;

private:
    vector<QPointF> m_data;    // All data points
    int m_visibleCount = 0;         // How many to draw

signals:
};

#endif // PLOTWIDGET_H

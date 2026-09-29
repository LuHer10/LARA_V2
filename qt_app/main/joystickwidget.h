#ifndef JOYSTICKWIDGET_H
#define JOYSTICKWIDGET_H

#define MAXRAD 80.0

#include <QWidget>
#include <QPointF>

class JoystickWidget : public QWidget
{
    Q_OBJECT

public:
    JoystickWidget(QWidget *parent = nullptr);
    QSize sizeHint() const override;

signals:
    void directionChanged(const float &x, const float &y);

protected:
    void paintEvent(QPaintEvent *) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    bool event(QEvent *event) override;


private:
    QPointF center;
    QPointF stick;
    //QPointF delta;
    bool dragging = false;

    void calcDelta();

};

#endif // JOYSTICKWIDGET_H

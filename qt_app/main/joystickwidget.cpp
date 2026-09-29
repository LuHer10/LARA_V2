#include "joystickwidget.h"
#include <QPainter>
#include <QMouseEvent>
#include <QTouchEvent>
#include <cmath>

JoystickWidget::JoystickWidget(QWidget *parent)
    : QWidget(parent)
{
    setMinimumSize(200, 200);
    setAttribute(Qt::WA_AcceptTouchEvents);
}

QSize JoystickWidget::sizeHint() const
{
    return QSize(200, 200);
}

void JoystickWidget::paintEvent(QPaintEvent *)
{
    QPainter p(this);
    center = QPointF(width() / 2, height() / 2);

    p.setBrush(Qt::lightGray);
    p.drawEllipse(center, MAXRAD, MAXRAD);  // Outer circle

    p.setBrush(Qt::darkGray);
    QPointF drawStick = dragging ? stick : center;
    p.drawEllipse(drawStick, 20, 20);  // Joystick knob
}

void JoystickWidget::mousePressEvent(QMouseEvent *event)
{
    dragging = true;
    QPointF delta = event->pos() - center;
    if (std::hypot(delta.x(), delta.y()) > MAXRAD) {
        delta = delta / std::hypot(delta.x(), delta.y()) * MAXRAD;
    }
    stick = center + delta;
    update();
    emit directionChanged(delta.x()/MAXRAD, delta.y()/MAXRAD);
}

void JoystickWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (dragging) {
        QPointF delta = event->pos() - center;
        if (std::hypot(delta.x(), delta.y()) > MAXRAD) {
            delta = delta / std::hypot(delta.x(), delta.y()) * MAXRAD;
        }
        stick = center + delta;
        update();
        emit directionChanged(delta.x()/MAXRAD, delta.y()/MAXRAD);
    }
}

void JoystickWidget::mouseReleaseEvent(QMouseEvent *)
{
    dragging = false;
    stick = center;
    update();
    emit directionChanged(0.0f, 0.0f);
}

bool JoystickWidget::event(QEvent *event)
{
    if (event->type() == QEvent::TouchBegin ||
        event->type() == QEvent::TouchUpdate ||
        event->type() == QEvent::TouchEnd)
    {
        QTouchEvent *touchEvent = static_cast<QTouchEvent *>(event);
        if (!touchEvent->points().isEmpty()) {
            QEventPoint p = touchEvent->points().first();
            QPointF delta = p.position() - center;
            if (std::hypot(delta.x(), delta.y()) > MAXRAD) {
                delta = delta / std::hypot(delta.x(), delta.y()) * MAXRAD;
            }
            stick = center + delta;
            dragging = (event->type() != QEvent::TouchEnd);
            update();
            ///QString coord = QString("%1,%2").arg(delta.x()/MAXRAD).arg(delta.y()/MAXRAD);
            emit directionChanged(delta.x()/MAXRAD, delta.y()/MAXRAD);
        }
        return true;
    }
    return QWidget::event(event);
}

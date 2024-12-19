#ifndef FRAME_H
#define FRAME_H

#include <QWidget>
#include <QPainter>
#include <QVector>
#include <QFile>
#include <QMessageBox>
#include <QtMath>
#include <QDebug>
#include <QMatrix4x4>
#include <QTransform>
#include <QRegularExpression>
#include <QMouseEvent>
#include <QWheelEvent>

class Frame : public QWidget
{
    Q_OBJECT
public:
    Frame(QWidget *parent = 0);
    bool upload(QString, QString);
    void setFiX(double initFiX) { FiX = initFiX * M_PI / 180; };
    void setFiY(double initFiY) { FiY = initFiY * M_PI / 180; };
    void setFiZ(double initFiZ) { FiZ = initFiZ * M_PI / 180; };
    void rotateX(bool b_repaint);
    void rotateY(bool b_repaint);
    void rotateZ(QString, bool b_repaint);
    void onRotateX(double turn);
    void onRotateY(double turn);
    void onRotateZ(double turn);
    void scaleX(double);
    void scaleY(double);
    void scaleZ(double);
    void moveToCoord(double, double, double);
    void reflectX();
    void reflectY();
    void reflectZ();
    void setOptionDraw(int value) { optionDraw = value; };
    void setOptionFill(bool value) { optionFill = value; };
    int getCanvasWidth();
    int getCanvasHeight();

protected:
    void paintEvent(QPaintEvent *) override;
    void mousePressEvent(QMouseEvent *event) override
    {
        if (event->button() == Qt::LeftButton)
        {
            _p = event->pos();
            rotationX = _p.x() - widthCanvas/2;
            rotationY = _p.y() - heightCanvas/2;
        }
    }
    void mouseMoveEvent(QMouseEvent *) override;

private:
    QString pathPolygons;
    QString pathPoints;
    int optionDraw = 0;
    int optionFill = false;
    static uint const widthCanvas = 700;
    static uint const heightCanvas = 700;
    QPoint _p;
    QImage screen;
    int buffFrame[widthCanvas][heightCanvas];
    double buffZ[widthCanvas][heightCanvas];
    double FiX = 0.07;
    double FiY = 0.07;
    double FiZ = 0.07;
    int rotationX;
    int rotationY;
    struct intCoord
    {
        int x, y;
        double z;
    };
    QVector3D lightCoord;
    QVector<QVector<int>> dataPolygons;
    QVector<QVector3D> dataPoints;
    void defaultDrawFigure(QPainter &painter);
    void drawFigureZBuffer(QPainter &painter);
    void drawFigureVeyler(QPainter &painter);
    bool fillingDataPolygons();
    bool fillingDataPoints();
    void rotateZLeft();
    void rotateZRight();
    void calculate(QMatrix4x4 &);
    void fillPolygon(int, QVector<intCoord> &);
    void customLine(int, intCoord &, intCoord &, QMap<int, QVector<intCoord>> &);
};
#endif

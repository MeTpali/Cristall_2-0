#include "Frame.h"
Frame::Frame(QWidget *parent) : QWidget(parent)
{
    screen = QImage(widthCanvas, heightCanvas, QImage::Format_ARGB32);
    lightCoord.setX(widthCanvas/2);
    lightCoord.setY(heightCanvas/2);
    lightCoord.setZ(50);
}

void Frame::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);

    QPainter painter(this); // Локальный QPainter
    QPen pen(Qt::black, 1, Qt::DashLine, Qt::SquareCap, Qt::RoundJoin);
    painter.setPen(pen);

    switch (optionDraw)
    {
    case 0:
        defaultDrawFigure(painter);
        break;
    case 1:
        drawFigureZBuffer(painter);
        break;
    case 2:
        drawFigureVeyler(painter);
        break;
    }
}

void Frame::defaultDrawFigure(QPainter &painter)
{
    for (int i = 0; i < dataPolygons.size(); i++)
        {
            QPointF points[dataPolygons[i].size()];

            for (int j = 0; j < dataPolygons[i].size(); j++)
            {
                points[j] = QPointF(dataPoints[dataPolygons[i][j]].x() + widthCanvas/2,
                                    dataPoints[dataPolygons[i][j]].y() + heightCanvas/2);
            }
            painter.drawPolygon(points, dataPolygons[i].size());
        }
}

void Frame::drawFigureZBuffer(QPainter &painter)
{
    screen.fill(QColor(Qt::white).rgb());
    for (uint x = 0; x < widthCanvas; x++)
    {
        for (uint y = 0; y < heightCanvas; y++)
        {
            buffFrame[x][y] = 0;
            buffZ[x][y] = -1000;
        }
    }
    for (int i = 0; i < dataPolygons.size(); i++)
    {
        QVector<intCoord> points;
        for (int j = 0; j < dataPolygons[i].size(); j++)
        {
            points.push_back({int(dataPoints[dataPolygons[i][j]].x() + widthCanvas/2 + 0.5),
                              int(dataPoints[dataPolygons[i][j]].y() + heightCanvas/2 + 0.5),
                              dataPoints[dataPolygons[i][j]].z()});
        }
        fillPolygon(i + 1, points);
    }
    painter.drawImage(1, 1, screen);
}
void Frame::drawFigureVeyler(QPainter &painter)
{

}
void Frame::fillPolygon(int idSegment, QVector<intCoord> &points)
{
    QMap<int, QVector<intCoord>> boundMap;
    for (int i = 0; i < points.size() - 1; customLine(idSegment, points[i], points[i + 1], boundMap), i++)
        ;
    customLine(idSegment, points.last(), points[0], boundMap);
    int distance = sqrt(pow((lightCoord.x() - widthCanvas/2), 2) + pow((lightCoord.y() - heightCanvas/2), 2) + pow(lightCoord.z(), 2));
    foreach (int key, boundMap.keys())
    {
        QVector<intCoord> value = boundMap.value(key);
        for (int i = value[0].y; i < value[1].y; i++)
        {
            if (buffFrame[key][i] != idSegment)
            {
                double tmp = value[0].z + double(value[1].z - value[0].z) * double(double(i - value[0].y) / double(value[1].y - value[0].y));
                if (tmp >= buffZ[key][i])
                {
                    buffFrame[key][i] = 0;
                    int tmpDistance = sqrt(pow((lightCoord.x() - key), 2) + pow((lightCoord.y() - i), 2) + pow((lightCoord.z() - tmp), 2));
                    double betweenDistance = distance - tmpDistance * 0.8;
                    int alpha = 255.0 * (1.0 - betweenDistance / 100.0) < 255 ? 255.0 * (1.0 - betweenDistance / 100.0) : 255;
                    alpha = alpha < 0 ? 0 : alpha;
                    switch (optionDraw)
                    {
                    case 1:
                        optionFill ? screen.setPixelColor(key, i, QColor(0, 20, 255, alpha)) : screen.setPixelColor(key, i, 4294967295); // Голубой или белый цвет.
                        break;
                    case 2:
                        optionFill ? screen.setPixelColor(key, i, QColor(0, 20, 255, alpha)) : screen.setPixelColor(key, i, 4294967295); // Голубой или белый цвет.
                        break;
                    }
                    buffZ[key][i] = tmp;
                }
            }
        }
    }
}
void Frame::customLine(int idSegment, intCoord &p1, intCoord &p2, QMap<int, QVector<intCoord>> &boundMap)
{
    const int deltaX = abs(p2.x - p1.x);
    const int deltaY = abs(p2.y - p1.y);
    const int signX = p1.x < p2.x ? 1 : -1;
    const int signY = p1.y < p2.y ? 1 : -1;
    int error = deltaX - deltaY;
    int x = p1.x, y = p1.y;
    double tmp;
    while (x != p2.x || y != p2.y)
    {
        if (p1.x == p2.x)
        {
            tmp = p1.z + double(p2.z - p1.z) * double(double(y - p1.y) / double(p2.y - p1.y));
        }
        else
        {
            tmp = p1.z + double(p2.z - p1.z) * double(double(x - p1.x) / double(p2.x - p1.x));
        }
        if (tmp >= buffZ[x][y])
        {
            buffFrame[x][y] = idSegment;
            screen.setPixelColor(x, y, 4278190080);
            buffZ[x][y] = tmp;
        }
        if (boundMap.find(x) == boundMap.end())
        {
            intCoord boundCoord = {y, static_cast<int>(tmp)};
            boundMap.insert(x, {boundCoord, boundCoord});
        }
        else
        {
            if (boundMap[x][0].y > y)
            {
                boundMap[x][0].y = y;
                boundMap[x][0].z = tmp;
            }
            else if (boundMap[x][1].y < y)
            {
                boundMap[x][1].y = y;
                boundMap[x][1].z = tmp;
            }
        }
        const int error2 = error * 2;
        if (error2 > -deltaY)
        {
            error -= deltaY;
            x += signX;
        }
        if (error2 < deltaX)
        {
            error += deltaX;
            y += signY;
        }
    }
}
void Frame::mouseMoveEvent(QMouseEvent *event)
{
    if (!_p.isNull())
    {
        if (abs(rotationX - (event->x() - int(widthCanvas / 2))) > 5 &&
            abs(rotationY - (event->y() - int(heightCanvas / 2))) > 5)
        {
            if (rotationX > (event->x() - int(widthCanvas / 2)))
            {
                if (rotationY > (event->y() - int(heightCanvas / 2)))
                {
                    if (FiX > 0)
                        FiX = -FiX;
                    rotateX(false);
                    FiY = abs(FiY);
                    rotateY(false);
                }
                else
                {
                    FiX = abs(FiX);
                    rotateX(false);
                    FiY = abs(FiY);
                    rotateY(false);
                }
            }
            else
            {
                if (rotationY > (event->y() - int(heightCanvas / 2)))
                {
                    if (FiX > 0)
                        FiX = -FiX;
                    rotateX(false);
                    if (FiY > 0)
                        FiY = -FiY;
                    rotateY(false);
                }
                else
                {
                    FiX = abs(FiX);
                    rotateX(false);
                    if (FiY > 0)
                        FiY = -FiY;
                    rotateY(false);
                }
            }
            rotationX = event->x() - int(widthCanvas / 2);
            rotationY = event->y() - int(heightCanvas / 2);
        }
    }
    repaint();
}
void Frame::rotateZLeft()
{
    rotateZ("Left", true);
}
void Frame::rotateZRight()
{
    rotateZ("Right", true);
}
void Frame::rotateX(bool b_repaint)
{
    QMatrix4x4 R(
        1, 0, 0, 0,
        0, cos(FiX), sin(FiX), 0,
        0, -sin(FiX), cos(FiX), 0,
        0, 0, 0, 1);
    calculate(R);
    if (b_repaint)
        repaint();
}
void Frame::rotateY(bool b_repaint)
{
    QMatrix4x4 R(
        cos(FiY), 0, -sin(FiY), 0,
        0, 1, 0, 0,
        sin(FiY), 0, cos(FiY), 0,
        0, 0, 0, 1);
    calculate(R);
    if (b_repaint)
        repaint();
}
void Frame::rotateZ(QString direction, bool b_repaint)
{
    if (direction == "Left")
    {
        FiZ = abs(FiZ);
    }
    else if (direction == "Right")
    {
        if (FiZ > 0)
            FiZ = -FiZ;
    }
    QMatrix4x4 R(
        cos(FiZ), sin(FiZ), 0, 0,
        -sin(FiZ), cos(FiZ), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1);
    calculate(R);
    if (b_repaint)
        repaint();
}
void Frame::onRotateX(double turn)
{
    double angle = turn * M_PI / 180;
    QMatrix4x4 R(
        1, 0, 0, 0,
        0, cos(angle), sin(angle), 0,
        0, -sin(angle), cos(angle), 0,
        0, 0, 0, 1);
    calculate(R);
    repaint();
}
void Frame::onRotateY(double turn)
{
    double angle = turn * M_PI / 180;
    QMatrix4x4 R(
        cos(angle), 0, -sin(angle), 0,
        0, 1, 0, 0,
        sin(angle), 0, cos(angle), 0,
        0, 0, 0, 1);
    calculate(R);
    repaint();
}
void Frame::onRotateZ(double turn)
{
    double angle = turn * M_PI / 180;
    QMatrix4x4 R(
        cos(angle), sin(angle), 0, 0,
        -sin(angle), cos(angle), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1);
    calculate(R);
    repaint();
}
void Frame::scaleX(double scaleValue)
{
    QMatrix4x4 R(
        scaleValue, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1);
    calculate(R);
    repaint();
}
void Frame::scaleY(double scaleValue)
{
    QMatrix4x4 R(
        1, 0, 0, 0,
        0, scaleValue, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1);
    calculate(R);
    repaint();
}
void Frame::scaleZ(double scaleValue)
{
    QMatrix4x4 R(
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, scaleValue, 0,
        0, 0, 0, 1);
    calculate(R);
    repaint();
}
void Frame::moveToCoord(double posX, double posY, double posZ)
{
    QMatrix4x4 R(
        1, 0, 0, posX,
        0, 1, 0, posY,
        0, 0, 1, posZ,
        0, 0, 0, 1);
    calculate(R);
    repaint();
}
void Frame::reflectX()
{
    QMatrix4x4 R(
        1, 0, 0, 0,
        0, -1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1);
    calculate(R);
    repaint();
}
void Frame::reflectY()
{
    QMatrix4x4 R(
        -1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1);
    calculate(R);
    repaint();
}
void Frame::reflectZ()
{
    QMatrix4x4 R(
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, -1, 0,
        0, 0, 0, 1);
    calculate(R);
    repaint();
}
void Frame::calculate(QMatrix4x4 &R)
{
    for (int i = 0; i < dataPoints.size(); i++)
    {
        QVector4D vectCoord(dataPoints[i].x(), dataPoints[i].y(), dataPoints[i].z(), 1);
        QVector4D result = R * vectCoord;
        dataPoints[i].setX(result[0]);
        dataPoints[i].setY(result[1]);
        dataPoints[i].setZ(result[2]);
    }
}
bool Frame::upload(QString initPathPolygons, QString initPathPoints)
{
    pathPolygons = initPathPolygons;
    pathPoints = initPathPoints;
    bool resultFillPolygons = fillingDataPolygons();
    bool resultFillPoints = fillingDataPoints();
    repaint();
    if (resultFillPolygons && resultFillPoints)
        return true;
    return false;
}
bool Frame::fillingDataPolygons()
{
    dataPolygons.clear();
    QFile file(pathPolygons);
    if (!file.open(QIODevice::ReadOnly))
    {
        QMessageBox::warning(this, "Предупреждение!", "Файл " + pathPolygons + " не найден!");
        return false;
    }
    while (!file.atEnd())
    {
        QStringList list;
        QString tmpStr = file.readLine();
        list = tmpStr.split(QRegularExpression(" "));
        QVector<int> tmpVec;
        for (int i = 1; i <= list.at(0).toInt(); i++)
        {
            tmpVec.push_back(list.at(i).toInt());
        }
        dataPolygons.push_back(tmpVec);
    }

    file.close();
    return true;
}
bool Frame::fillingDataPoints()
{
    dataPoints.clear();
    QFile file(pathPoints);
    if (!file.open(QIODevice::ReadOnly))
    {
        QMessageBox::warning(this, "Предупреждение!", "Файл " + pathPoints + " не найден!");
        return false;
    }
    while (!file.atEnd())
    {
        QStringList list;
        QString tmp = file.readLine();
        list = tmp.split(QRegularExpression(" "));
        dataPoints.push_back({100 * list[0].toFloat(), 100 * list[1].toFloat(), 100 * list[2].toFloat()});
    }

    file.close();
}

int Frame::getCanvasWidth(){
    return widthCanvas;
}

int Frame::getCanvasHeight(){
    return heightCanvas;
}

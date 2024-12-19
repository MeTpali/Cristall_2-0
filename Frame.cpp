#include "Frame.h"
Frame::Frame(QWidget *parent) : QWidget(parent)
{
    screen = QImage(widthCanvas, heightCanvas, QImage::Format_ARGB32);
    lightCoord.setX(widthCanvas / 2);
    lightCoord.setY(heightCanvas / 2);
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
    case 3:
        drawFigureGuro(painter);
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
            points[j] = QPointF(dataPoints[dataPolygons[i][j]].x() + widthCanvas / 2,
                                dataPoints[dataPolygons[i][j]].y() + heightCanvas / 2);
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
            points.push_back({int(dataPoints[dataPolygons[i][j]].x() + widthCanvas / 2 + 0.5),
                              int(dataPoints[dataPolygons[i][j]].y() + heightCanvas / 2 + 0.5),
                              dataPoints[dataPolygons[i][j]].z()});
        }

        fillPolygon(i + 1, points);
    }

    painter.drawImage(1, 1, screen);
}

void Frame::fillPolygon(int idSegment, QVector<intCoord> &points)
{
    QMap<int, QVector<intCoord>> boundMap;
    for (int i = 0; i < points.size() - 1; customLine(idSegment, points[i], points[i + 1], boundMap), i++)
        ;
    customLine(idSegment, points.last(), points[0], boundMap);
    int distance = sqrt(pow((lightCoord.x() - widthCanvas / 2), 2) + pow((lightCoord.y() - heightCanvas / 2), 2) + pow(lightCoord.z(), 2));
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
                    if (key >= 0 && key < widthCanvas && i >= 0 && i < heightCanvas)
                    {
                        screen.setPixel(key, i, 4294967295);
                    }
                    buffZ[key][i] = tmp;
                }
            }
        }
    }
}
void Frame::customLine(int idSegment, intCoord &p1, intCoord &p2, QMap<int, QVector<intCoord>> &boundMap)
{
    if (p1.x < 0 || p1.x >= widthCanvas || p1.y < 0 || p1.y >= heightCanvas ||
        p2.x < 0 || p2.x >= widthCanvas || p2.y < 0 || p2.y >= heightCanvas)
    {
        return;
    }

    const int deltaX = abs(p2.x - p1.x);
    const int deltaY = abs(p2.y - p1.y);
    const int signX = p1.x < p2.x ? 1 : -1;
    const int signY = p1.y < p2.y ? 1 : -1;
    int error = deltaX - deltaY;
    int x = p1.x, y = p1.y;
    double tmp;

    while (x >= 0 && x < widthCanvas && y >= 0 && y < heightCanvas && (x != p2.x || y != p2.y))
    {
        if (p1.x == p2.x)
        {
            if (p1.y != p2.y)
            {
                tmp = p1.z + double(p2.z - p1.z) * double(double(y - p1.y) / double(p2.y - p1.y));
            }
            else
            {
                tmp = p1.z;
            }
        }
        else
        {
            tmp = p1.z + double(p2.z - p1.z) * double(double(x - p1.x) / double(p2.x - p1.x));
        }

        if (x >= 0 && x < widthCanvas && y >= 0 && y < heightCanvas)
        {
            if (tmp >= buffZ[x][y])
            {
                buffFrame[x][y] = idSegment;
                screen.setPixelColor(x, y, 4278190080);
                buffZ[x][y] = tmp;
            }
        }

        if (y >= 0 && y < heightCanvas)
        {
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

// Функция для вычисления Z-координаты точки (x, y) на плоскости, заданной тремя точками a, b, c
double getPlaneZCoord(double x, double y, const QVector3D &a, const QVector3D &b, const QVector3D &c)
{
    // Векторы на плоскости
    QVector3D AB = b - a;
    QVector3D AC = c - a;

    // Нормаль к плоскости
    QVector3D normal = QVector3D::crossProduct(AB, AC);

    // Проверка, чтобы нормаль не была параллельна оси Z (избегаем деления на ноль)
    if (std::abs(normal.z()) < 1e-7)
    {
        // Плоскость параллельна оси Z, возвращаем большое отрицательное значение
        return -1e9;
    }

    // Уравнение плоскости: Ax + By + Cz + D = 0
    double A = normal.x();
    double B = normal.y();
    double C = normal.z();
    double D = -QVector3D::dotProduct(normal, a);

    // Вычисляем Z для заданных X и Y
    double z = (-A * x - B * y - D) / C;
    return z;
}

// Структура для хранения индекса полигона и его средней Z-координаты
struct PolyZ
{
    int idx;     // Индекс полигона
    double avgZ; // Средняя Z-координата полигона
};

void Frame::drawFigureVeyler(QPainter &painter)
{
    // Очищаем буфер вывода цветом белого фона
    screen.fill(QColor(Qt::white).rgb());
    QBrush brush(Qt::white);          // Задаём синий цвет заливки
    brush.setStyle(Qt::SolidPattern); // Задаём сплошную заливку
    painter.setBrush(brush);

    // Создаём вектор структур PolyZ для всех полигонов
    QVector<PolyZ> ZCoords;
    ZCoords.reserve(dataPolygons.size());

    // Вычисление средней координаты Z каждого полигона
    for (size_t i = 0; i < dataPolygons.size(); ++i)
    {
        const auto &polygon = dataPolygons[i];
        double sumZ = 0.0;
        int count = 0;

        // Суммируем Z-координаты всех вершин полигона
        for (const auto &pointIdx : polygon)
        {
            const QVector3D &point = dataPoints[pointIdx];
            sumZ += point.z();
            ++count;
        }

        double averageZ = (count > 0) ? (sumZ / count) : 0.0;

        ZCoords.push_back(PolyZ{static_cast<int>(i), averageZ});
    }

    // Сортируем полигоны по средней Z-координате в порядке убывания (от дальних к ближним)
    std::sort(ZCoords.begin(), ZCoords.end(),
              [](const PolyZ &a, const PolyZ &b) -> bool
              {
                  return a.avgZ < b.avgZ; // От меньшего к большему (дальние сначала)
              });

    // Отрисовываем полигоны от дальних к ближним
    for (const auto &polyZ : ZCoords)
    {
        int idx = polyZ.idx;
        const auto &polygon = dataPolygons[idx];

        // Создаём массив точек для полигона с учётом смещения центра экрана
        QVector<QPointF> points;
        points.reserve(polygon.size());
        for (const auto &pointIdx : polygon)
        {
            const QVector3D &point = dataPoints[pointIdx];
            // Предполагается, что X и Y уже находятся в системе координат экрана
            QPointF screenPoint(point.x() + widthCanvas / 2.0,
                                point.y() + heightCanvas / 2.0);
            points.append(screenPoint);
        }

        // Отрисовываем текущий полигон
        painter.drawPolygon(points.data(), points.size());
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
    if (resultFillPolygons && resultFillPoints)
        return true;
    repaint();
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

int Frame::getCanvasWidth()
{
    return widthCanvas;
}

int Frame::getCanvasHeight()
{
    return heightCanvas;
}
// Структура для хранения нормали полигона
struct PolygonNormal
{
    QVector3D normal;
};

// Структура для хранения нормали вершины
struct VertexNormal
{
    QVector3D normal;
};

// Структура для хранения интенсивности вершины
struct VertexIntensity
{
    double intensity;
};

// Структура для хранения треугольника с интенсивностями
struct ShadedTriangle
{
    QVector3D v0;
    QVector3D v1;
    QVector3D v2;
    double i0;
    double i1;
    double i2;
};

// Функция для вычисления нормали треугольника
QVector3D computeNormal(const QVector3D &a, const QVector3D &b, const QVector3D &c)
{
    QVector3D u = b - a;
    QVector3D v = c - a;
    QVector3D normal = QVector3D::crossProduct(u, v).normalized();
    return normal;
}

// Функция для вычисления нормалей всех полигонов
QVector<PolygonNormal> computePolygonNormals(const QVector<QVector<int>> &dataPolygons, const QVector<QVector3D> &dataPoints)
{
    QVector<PolygonNormal> polygonNormals;
    polygonNormals.reserve(dataPolygons.size());

    for (const auto &poly : dataPolygons)
    {
        if (poly.size() < 3)
        {
            // Некорректный полигон, добавляем нулевую нормаль
            polygonNormals.append(PolygonNormal{QVector3D(0, 0, 0)});
            continue;
        }

        // Выбираем первые три вершины для вычисления нормали
        QVector3D a = dataPoints[poly[0]];
        QVector3D b = dataPoints[poly[1]];
        QVector3D c = dataPoints[poly[2]];

        QVector3D normal = computeNormal(a, b, c);
        polygonNormals.append(PolygonNormal{normal});
    }

    return polygonNormals;
}

// Функция для вычисления нормалей вершин
QVector<VertexNormal> computeVertexNormals(const QVector<QVector<int>> &dataPolygons, const QVector<PolygonNormal> &polygonNormals, int numVertices)
{
    QVector<QVector3D> tempNormals(numVertices, QVector3D(0, 0, 0));
    QVector<int> counts(numVertices, 0);

    // Суммируем нормали всех полигонов, которым принадлежит вершина
    for (size_t i = 0; i < dataPolygons.size(); ++i)
    {
        const auto &poly = dataPolygons[i];
        const QVector3D &normal = polygonNormals[i].normal;

        for (int vertexIdx : poly)
        {
            tempNormals[vertexIdx] += normal;
            counts[vertexIdx]++;
        }
    }

    // Усредняем нормали
    QVector<VertexNormal> vertexNormals;
    vertexNormals.reserve(numVertices);

    for (int i = 0; i < numVertices; ++i)
    {
        if (counts[i] > 0)
        {
            QVector3D avgNormal = tempNormals[i] / static_cast<float>(counts[i]);
            avgNormal.normalize();
            vertexNormals.append(VertexNormal{avgNormal});
        }
        else
        {
            // Если вершина не принадлежит ни одному полигону, устанавливаем нулевую нормаль
            vertexNormals.append(VertexNormal{QVector3D(0, 0, 0)});
        }
    }

    return vertexNormals;
}

// Функция для вычисления интенсивности в вершинах
QVector<VertexIntensity> computeVertexIntensities(const QVector<VertexNormal> &vertexNormals, const QVector3D &lightDirection)
{
    QVector<VertexIntensity> vertexIntensities;
    vertexIntensities.reserve(vertexNormals.size());

    QVector3D normalizedLightDir = lightDirection.normalized();

    for (const auto &vNormal : vertexNormals)
    {
        // Диффузное освещение: I = max(0, N • L)
        double intensity = QVector3D::dotProduct(vNormal.normal, normalizedLightDir);
        intensity = std::max(0.0, static_cast<double>(intensity));
        vertexIntensities.append(VertexIntensity{intensity});
    }

    return vertexIntensities;
}

// Функция для триангуляции многоугольника (предполагается выпуклый)
QVector<ShadedTriangle> triangulateAndShade(const QVector<QVector3D> &polygonVertices, const QVector<VertexIntensity> &vertexIntensities)
{
    QVector<ShadedTriangle> shadedTriangles;
    if (polygonVertices.size() < 3)
        return shadedTriangles;
    // Используем первую вершину как базовую
    QVector3D base = polygonVertices[0];
    double iBase = vertexIntensities[0].intensity;

    for (int i = 1; i < polygonVertices.size() - 1; ++i)
    {
        ShadedTriangle tri;
        tri.v0 = base;
        tri.v1 = polygonVertices[i];
        tri.v2 = polygonVertices[i + 1];
        tri.i0 = iBase;
        tri.i1 = vertexIntensities[i].intensity;
        tri.i2 = vertexIntensities[i + 1].intensity;
        shadedTriangles.append(tri);
    }

    return shadedTriangles;
}

// Функция для интерполяции интенсивности по барицентрическим координатам
double interpolateIntensity(double u, double v, double w, double i0, double i1, double i2)
{
    return u * i0 + v * i1 + w * i2;
}

// Функция для закраски методом Гуро
void drawFigureGuro(QPainter &painter, const QVector<QVector<int>> &dataPolygons, const QVector<QVector3D> &dataPoints, const QVector<VertexIntensity> &vertexIntensities, int widthCanvas, int heightCanvas)
{
    // Создаём буфер изображения
    QImage screen(widthCanvas, heightCanvas, QImage::Format_RGB32);
    screen.fill(Qt::black);

    // Создаём Z-буфер, инициализируем его минимально возможными значениями
    QVector<QVector<double>> zBuffer(widthCanvas, QVector<double>(heightCanvas, -std::numeric_limits<double>::infinity()));

    // Создаём вектор треугольников с интенсивностями
    QVector<ShadedTriangle> shadedTriangles;

    for (const auto &poly : dataPolygons)
    {
        if (poly.size() < 3)
            continue; // Пропускаем некорректные полигоны

        // Собираем вершины полигона
        QVector<QVector3D> polygonVertices;
        QVector<VertexIntensity> polygonIntensities;
        for (int idx : poly)
        {
            QVector3D point = dataPoints[idx];
            // Смещаем координаты относительно центра канвы
            point.setX(point.x() + widthCanvas / 2.0);
            point.setY(point.y() + heightCanvas / 2.0);
            polygonVertices.append(point);
            polygonIntensities.append(vertexIntensities[idx]);
        }

        // Триангулируем многоугольник и получаем закрашенные треугольники
        QVector<ShadedTriangle> tris = triangulateAndShade(polygonVertices, polygonIntensities);
        shadedTriangles.append(tris);
    }

    // Растеризация каждого закрашенного треугольника
    for (const auto &tri : shadedTriangles)
    {
        // Определяем границы треугольника
        int minX = std::floor(std::min({tri.v0.x(), tri.v1.x(), tri.v2.x()}));
        int maxX = std::ceil(std::max({tri.v0.x(), tri.v1.x(), tri.v2.x()}));
        int minY = std::floor(std::min({tri.v0.y(), tri.v1.y(), tri.v2.y()}));
        int maxY = std::ceil(std::max({tri.v0.y(), tri.v1.y(), tri.v2.y()}));

        // Ограничиваем границы экрана
        minX = std::max(minX, 0);
        maxX = std::min(maxX, widthCanvas - 1);
        minY = std::max(minY, 0);
        maxY = std::min(maxY, heightCanvas - 1);

        // Проходим по каждому пикселю в ограничивающем прямоугольнике
        for (int y = minY; y <= maxY; y++)
        {
            for (int x = minX; x <= maxX; x++)
            {
                // Используем центр пикселя для проверки принадлежности
                double px = x + 0.5;
                double py = y + 0.5;

                // Вычисляем барицентрические координаты
                double denom = (tri.v1.y() - tri.v2.y()) * (tri.v0.x() - tri.v2.x()) +
                               (tri.v2.x() - tri.v1.x()) * (tri.v0.y() - tri.v2.y());

                if (std::abs(denom) < 1e-9)
                    continue; // Треугольник вырожден

                double u = ((tri.v1.y() - tri.v2.y()) * (px - tri.v2.x()) +
                            (tri.v2.x() - tri.v1.x()) * (py - tri.v2.y())) /
                           denom;
                double v = ((tri.v2.y() - tri.v0.y()) * (px - tri.v2.x()) +
                            (tri.v0.x() - tri.v2.x()) * (py - tri.v2.y())) /
                           denom;
                double w = 1.0 - u - v;
                // Проверяем, принадлежит ли пиксель треугольнику
                if (u >= 0 && v >= 0 && w >= 0)
                {
                    // Интерполируем Z
                    double z = u * tri.v0.z() + v * tri.v1.z() + w * tri.v2.z();

                    // Проверяем Z-буфер
                    if (z > zBuffer[x][y])
                    { // Предполагаем, что большее Z ближе
                        zBuffer[x][y] = z;

                        // Интерполируем интенсивность
                        double intensity = interpolateIntensity(u, v, w, tri.i0, tri.i1, tri.i2);
                        intensity = std::clamp(intensity, 0.0, 1.0); // Ограничиваем значения

                        // Преобразуем интенсивность в оттенки серого
                        int gray = static_cast<int>(intensity * 255);
                        QColor color(gray, gray, gray);

                        // Устанавливаем цвет пикселя
                        screen.setPixelColor(x, y, color);
                    }
                }
            }
        }
    }

    // Отображаем готовое изображение
    painter.drawImage(0, 0, screen);
}
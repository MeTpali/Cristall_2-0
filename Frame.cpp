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
                                if (key >= 0 && key < widthCanvas && i >= 0 && i < heightCanvas) {
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
        p2.x < 0 || p2.x >= widthCanvas || p2.y < 0 || p2.y >= heightCanvas) {
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
        if (p1.x == p2.x) {
            if (p1.y != p2.y) {
                tmp = p1.z + double(p2.z - p1.z) * double(double(y - p1.y) / double(p2.y - p1.y));
            } else {
                tmp = p1.z;
            }
        } else {
            tmp = p1.z + double(p2.z - p1.z) * double(double(x - p1.x) / double(p2.x - p1.x));
        }

        if (x >= 0 && x < widthCanvas && y >= 0 && y < heightCanvas) {
            if (tmp >= buffZ[x][y]) {
                buffFrame[x][y] = idSegment;
                screen.setPixelColor(x, y, 4278190080);
                buffZ[x][y] = tmp;
            }
        }

        if (y >= 0 && y < heightCanvas) {
            if (boundMap.find(x) == boundMap.end()) {
                intCoord boundCoord = {y, static_cast<int>(tmp)};
                boundMap.insert(x, {boundCoord, boundCoord});
            } else {
                if (boundMap[x][0].y > y) {
                    boundMap[x][0].y = y;
                    boundMap[x][0].z = tmp;
                } else if (boundMap[x][1].y < y) {
                    boundMap[x][1].y = y;
                    boundMap[x][1].z = tmp;
                }
            }
        }

        const int error2 = error * 2;
        if (error2 > -deltaY) {
            error -= deltaY;
            x += signX;
        }
        if (error2 < deltaX) {
            error += deltaX;
            y += signY;
        }
    }
}

// Функция для вычисления Z-координаты точки (x, y) на плоскости, заданной тремя точками a, b, c
double getPlaneZCoord(double x, double y, const QVector3D& a, const QVector3D& b, const QVector3D& c)
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
    int idx;       // Индекс полигона
    double avgZ;   // Средняя Z-координата полигона
};

void Frame::drawFigureVeyler(QPainter &painter)
{
    // Очищаем буфер вывода цветом белого фона
    screen.fill(QColor(Qt::white).rgb());
    QBrush brush(Qt::white); // Задаём синий цвет заливки
    brush.setStyle(Qt::SolidPattern); // Задаём сплошную заливку
    painter.setBrush(brush);

    // Создаём вектор структур PolyZ для всех полигонов
    QVector<PolyZ> ZCoords;
    ZCoords.reserve(dataPolygons.size());

    // Вычисление средней координаты Z каждого полигона
    for (size_t i = 0; i < dataPolygons.size(); ++i)
    {
        const auto& polygon = dataPolygons[i];
        double sumZ = 0.0;
        int count = 0;

        // Суммируем Z-координаты всех вершин полигона
        for (const auto& pointIdx : polygon)
        {
            const QVector3D& point = dataPoints[pointIdx];
            sumZ += point.z();
            ++count;
        }

        double averageZ = (count > 0) ? (sumZ / count) : 0.0;

        ZCoords.push_back(PolyZ{ static_cast<int>(i), averageZ });
    }

    // Сортируем полигоны по средней Z-координате в порядке убывания (от дальних к ближним)
    std::sort(ZCoords.begin(), ZCoords.end(),
              [](const PolyZ& a, const PolyZ& b) -> bool
              {
                  return a.avgZ < b.avgZ; // От меньшего к большему (дальние сначала)
              });

    // Отрисовываем полигоны от дальних к ближним
    for (const auto& polyZ : ZCoords)
    {
        int idx = polyZ.idx;
        const auto& polygon = dataPolygons[idx];

        // Создаём массив точек для полигона с учётом смещения центра экрана
        QVector<QPointF> points;
        points.reserve(polygon.size());
        for (const auto& pointIdx : polygon)
        {
            const QVector3D& point = dataPoints[pointIdx];
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

int Frame::getCanvasWidth(){
    return widthCanvas;
}

int Frame::getCanvasHeight(){
    return heightCanvas;
}

// Структура для хранения треугольника
struct Triangle {
    QVector3D v0;
    QVector3D v1;
    QVector3D v2;
};

// Функция триангуляции выпуклого многоугольника
QVector<Triangle> triangulateConvexPolygon(const QVector<QVector3D> &polygon) {
    QVector<Triangle> triangles;
    if (polygon.size() < 3) return triangles; // Некорректный многоугольник

    // Используем первую вершину как базовую
    QVector3D base = polygon[0];
    for (int i = 1; i < polygon.size() - 1; ++i) {
        Triangle tri;
        tri.v0 = base;
        tri.v1 = polygon[i];
        tri.v2 = polygon[i + 1];
        triangles.append(tri);
    }
    return triangles;
}

QColor getColorByZ(double z, double minZ, double maxZ)
{
    // Нормализуем Z в диапазон [0, 1]
    double normalizedZ = (z - minZ) / (maxZ - minZ);
    normalizedZ = std::clamp(normalizedZ, 0.0, 1.0); // Ограничиваем значения

    // Инвертируем нормализованное значение, чтобы ближайшие были светлее
    double brightness = normalizedZ;

    // Преобразуем яркость в оттенки серого
    int gray = static_cast<int>(brightness * 255);
    return QColor(gray, gray, gray);
}

void colorPixel(int x, int y, double z, QImage &image, QVector<QVector<double>> &zBuffer, double minZ, double maxZ)
{
    // Проверяем границы экрана
    if (x < 0 || x >= image.width() || y < 0 || y >= image.height())
        return;

    // Проверяем Z-буфер
    if (z > zBuffer[x][y])
    {
        zBuffer[x][y] = z;
        QColor color = getColorByZ(z, minZ, maxZ);
        image.setPixelColor(x, y, color);
    }
}

bool isPointInTriangle(double px, double py, const Triangle &triangle, double &u, double &v, double &w)
{
    double x0 = triangle.v0.x();
    double y0 = triangle.v0.y();
    double x1 = triangle.v1.x();
    double y1 = triangle.v1.y();
    double x2 = triangle.v2.x();
    double y2 = triangle.v2.y();

    double denom = (y1 - y2)*(x0 - x2) + (x2 - x1)*(y0 - y2);
    if (std::abs(denom) < 1e-9)
        return false;

    u = ((y1 - y2)*(px - x2) + (x2 - x1)*(py - y2)) / denom;
    v = ((y2 - y0)*(px - x2) + (x0 - x2)*(py - y2)) / denom;
    w = 1 - u - v;

    return (u >= 0) && (v >= 0) && (w >= 0);
}



    // Функция отрисовки фигур с по-пиксельным закрашиванием
    void Frame::drawFigureGuro(QPainter &painter)
    {

          QVector<QVector<double>> zBuffer;
           screen.fill(Qt::white);

           zBuffer.resize(widthCanvas, QVector<double>(heightCanvas, -std::numeric_limits<double>::infinity()));

           // Найдём минимальные и максимальные значения Z среди всех полигонов
           double minZ = std::numeric_limits<double>::max();
           double maxZ = std::numeric_limits<double>::lowest();

           // Создадим вектор треугольников
           QVector<Triangle> triangles;

           for (const auto &poly : dataPolygons) {
               if (poly.size() < 3)
                   continue; // Пропускаем некорректные полигоны

               // Собираем вершины полигона
               QVector<QVector3D> polygonVertices;
               for (int idx : poly) {
                   // Смещаем координаты относительно центра канвы
                   QVector3D point = dataPoints[idx];
                   point.setX(point.x() + widthCanvas / 2.0);
                   point.setY(point.y() + heightCanvas / 2.0);
                   polygonVertices.append(point);
               }

               // Триангулируем многоугольник (предполагается выпуклый)
               QVector<Triangle> tris = triangulateConvexPolygon(polygonVertices);
               triangles.append(tris);

               // Обновляем minZ и maxZ
               for (const auto &tri : tris) {
                minZ = *std::min_element(
                    std::initializer_list<double>{minZ, tri.v0.z(), tri.v1.z(), tri.v2.z()}.begin(),
                    std::initializer_list<double>{minZ, tri.v0.z(), tri.v1.z(), tri.v2.z()}.end());

                maxZ = *std::max_element(
                    std::initializer_list<double>{maxZ, tri.v0.z(), tri.v1.z(), tri.v2.z()}.begin(),
                std::initializer_list<double>{maxZ, tri.v0.z(), tri.v1.z(), tri.v2.z()}.end());

               }
           }

           // Растеризация каждого треугольника
           for (const auto &tri : triangles) {
               // Определяем границы треугольника
               int minX = std::floor(std::min({tri.v0.x(), tri.v1.x(), tri.v2.x()}));
               int maxX = std::ceil(std::max({tri.v0.x(), tri.v1.x(), tri.v2.x()}));
               int minY = std::floor(std::min({tri.v0.y(), tri.v1.y(), tri.v2.y()}));
               int maxY = std::ceil(std::max({tri.v0.y(), tri.v1.y(), tri.v2.y()}));

               // Ограничиваем границы экрана
               minX = std::max(minX, 0);
               maxX = std::min(maxX, static_cast<int>(widthCanvas) - 1);
               minY = std::max(minY, 0);
               maxY = std::min(maxY, static_cast<int>(heightCanvas) - 1);


               // Проходим по каждому пикселю в ограничивающем прямоугольнике
               for (int y = minY; y <= maxY; y++) {
                   for (int x = minX; x <= maxX; x++) {
                       double u, v, w;
                       // Используем центр пикселя для проверки принадлежности
                       if (isPointInTriangle(x + 0.5, y + 0.5, tri, u, v, w)) {
                           // Интерполируем Z
                           double z = u * tri.v0.z() + v * tri.v1.z() + w * tri.v2.z();
                           // Закрашиваем пиксель
                           colorPixel(x, y, z, screen, zBuffer, minZ, maxZ);
                       }
                   }
               }
           }

           // Отображаем готовое изображение
           painter.drawImage(0, 0, screen);
    }

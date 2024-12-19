#ifndef CRISTALL_H
#define CRISTALL_H
#include <QMainWindow>
#include <QFileDialog>
#include <QMessageBox>
#include <QString>
#include <QPalette>
#include <Frame.h>
QT_BEGIN_NAMESPACE
namespace Ui
{
    class Cristall;
}
QT_END_NAMESPACE
class Cristall : public QMainWindow
{
    Q_OBJECT
public:
    Cristall(QWidget *parent = nullptr);
    ~Cristall();
private slots:
    void slotAddPolygons();
    void slotAddPoints();
    void slotUploadFiles();
//    void slotExit();
    void lineEditTurnChange();
    void onTurnX();
    void onTurnY();
    void onTurnZ();
    void onScaleX();
    void onScaleY();
    void onScaleZ();
    void onMove();
    void onReflectX();
    void onReflectY();
    void onReflectZ();
    void slotZBuffer(int);
    void slotVeyler(int);
    void slotGuro(int);

private:
    Ui::Cristall *ui;
    QString pathPolygons;
    QString pathPoints;
    Frame *canvas;
};
#endif

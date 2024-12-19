#include "Cristall.h"
#include "ui_Cristall.h"
Cristall::Cristall(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::Cristall)
{
    ui->setupUi(this);
    connect(ui->selectVerts, SIGNAL(clicked()), this, SLOT(slotAddPoints()));
    connect(ui->selectPolys, SIGNAL(clicked()), this, SLOT(slotAddPolygons()));
    connect(ui->pushButtonLoadFiles, SIGNAL(clicked()), this, SLOT(slotUploadFiles()));
    connect(ui->pushButtonReflectX, SIGNAL(clicked()), this, SLOT(onReflectX()));
    connect(ui->pushButtonReflectY, SIGNAL(clicked()), this, SLOT(onReflectY()));
    connect(ui->pushButtonReflectZ, SIGNAL(clicked()), this, SLOT(onReflectZ()));
    connect(ui->pushButtonScaleX, SIGNAL(clicked()), this, SLOT(onScaleX()));
    connect(ui->pushButtonScaleY, SIGNAL(clicked()), this, SLOT(onScaleY()));
    connect(ui->pushButtonScaleZ, SIGNAL(clicked()), this, SLOT(onScaleZ()));
    connect(ui->pushButtonMove, SIGNAL(clicked()), this, SLOT(onMove()));
    connect(ui->pushButtonTurnX, SIGNAL(clicked()), this, SLOT(onTurnX()));
    connect(ui->pushButtonTurnY, SIGNAL(clicked()), this, SLOT(onTurnY()));
    connect(ui->pushButtonTurnZ, SIGNAL(clicked()), this, SLOT(onTurnZ()));
//    connect(ui->exit, SIGNAL(triggered()), this, SLOT(slotExit()));
    connect(ui->checkBoxZBuffer, SIGNAL(stateChanged(int)), this, SLOT(slotZBuffer(int)));
    connect(ui->checkBoxVeyler, SIGNAL(stateChanged(int)), this, SLOT(slotVeyler(int)));
    connect(ui->checkBoxGuro, SIGNAL(stateChanged(int)), this, SLOT(slotGuro(int)));


    canvas = new Frame(this);
    QPalette palette = canvas->palette();
    palette.setColor(QPalette::Window, Qt::white); // Используем Window вместо Background
    canvas->setPalette(palette);
    canvas->setAutoFillBackground(true);
    canvas->setMinimumSize(canvas->getCanvasWidth(), canvas->getCanvasHeight()); // Укажите разумные размеры
    ui->canvasLayout->addWidget(canvas);
}
Cristall::~Cristall()
{
    canvas->deleteLater();
    delete ui;
}
void Cristall::slotAddPolygons()
{
    QString tmp = QFileDialog::getOpenFileName(0, "Choose file with polygons", "", "*.txt");
    if (tmp.isEmpty())
    {
        QMessageBox::warning(this, "Внимание!", "Вы не выбрали файл!");
        return;
    }
    else
    {
        pathPolygons = tmp;
        ui->uiPathPolygons->setText(pathPolygons);
        ui->pushButtonLoadFiles->setText("Загрузить");
    }
}
void Cristall::slotAddPoints()
{
    QString tmp = QFileDialog::getOpenFileName(0, "Choose file with points", "", "*.txt");
    if (tmp.isEmpty())
    {
        QMessageBox::warning(this, "Внимание!", "Вы не выбрали файл!");
        return;
    }
    else
    {
        pathPoints = tmp;
        ui->uiPathPoints->setText(pathPoints);
        ui->pushButtonLoadFiles->setText("Загрузить");
    }
}
void Cristall::slotUploadFiles()
{
    if (pathPolygons.isEmpty())
    {
        QMessageBox::warning(this, "Внимание!", "Вы не выбрали файл полигонов!");
        return;
    }
    if (pathPoints.isEmpty())
    {
        QMessageBox::warning(this, "Внимание!", "Вы не выбрали файл вершин!");
        return;
    }
    bool resultUpload = canvas->upload(pathPolygons, pathPoints);
    if (resultUpload)
    {
        ui->pushButtonLoadFiles->setText("Сбросить");
    }
}
//void Cristall::slotExit()
//{
//    this->close();
//}
void Cristall::lineEditTurnChange()
{
    canvas->setFiX(ui->lineEditTurnX->text().toDouble());
    canvas->setFiY(ui->lineEditTurnY->text().toDouble());
    canvas->setFiZ(ui->lineEditTurnZ->text().toDouble());
}
void Cristall::onTurnX()
{
    canvas->onRotateX(ui->lineEditTurnX->text().toDouble());
}
void Cristall::onTurnY()
{
    canvas->onRotateY(ui->lineEditTurnY->text().toDouble());
}
void Cristall::onTurnZ()
{
    canvas->onRotateZ(ui->lineEditTurnZ->text().toDouble());
}
void Cristall::onScaleX()
{
    canvas->scaleX(ui->lineEditScaleX->text().toDouble());
}
void Cristall::onScaleY()
{
    canvas->scaleY(ui->lineEditScaleY->text().toDouble());
}
void Cristall::onScaleZ()
{
    canvas->scaleZ(ui->lineEditScaleZ->text().toDouble());
}
void Cristall::onMove()
{
    canvas->moveToCoord(ui->lineEditMoveX->text().toDouble(), ui->lineEditMoveY->text().toDouble(), ui->lineEditMoveZ->text().toDouble());
}
void Cristall::onReflectX()
{
    canvas->reflectX();
}
void Cristall::onReflectY()
{
    canvas->reflectY();
}
void Cristall::onReflectZ()
{
    canvas->reflectZ();
}
void Cristall::slotZBuffer(int check)
{
    if (check == 2)
    {
        canvas->setOptionDraw(1);
        ui->checkBoxZBuffer->setCheckState(Qt::Checked);
        ui->checkBoxVeyler->setCheckState(Qt::Unchecked);
    }
    else
    {
        if (!ui->checkBoxVeyler->isChecked())
        {
            canvas->setOptionDraw(0);
        }
    }
    canvas->update();
}
void Cristall::slotVeyler(int check)
{
    if (check == 2)
    {
        canvas->setOptionDraw(2);
        ui->checkBoxZBuffer->setCheckState(Qt::Unchecked);
        ui->checkBoxVeyler->setCheckState(Qt::Checked);
    }
    else
    {
        if (!ui->checkBoxZBuffer->isChecked())
        {
            canvas->setOptionDraw(0);
        }
    }
    canvas->update();
}
void Cristall::slotGuro(int check)
{
    if (check == 2)
    {
        canvas->setOptionFill(true);
    }
    else
    {
        canvas->setOptionFill(false);
    }
    canvas->update();
}

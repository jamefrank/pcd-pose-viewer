#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->splitter->setStretchFactor(0, 3);
    ui->splitter->setStretchFactor(1, 20);
}

MainWindow::~MainWindow()
{
    delete ui;
}


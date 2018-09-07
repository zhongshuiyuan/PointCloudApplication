//
// Created by WuKun on 9/4/18.
// Contact me:wk707060335@gmail.com
//

#include <iostream>
#include <string>
#include <algorithm>

#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QVBoxLayout>
#include <QtCore/QVector>
#include <QtCore/QDebug>

#include "EditorDialog.h"
#include "../Common/common.h"

EditorDialog::EditorDialog(QStringList& itemInfo, QWidget *parent) :
    QDialog(parent),
    type_(0),
    itemInfo_(itemInfo){
    initUI();
}

EditorDialog::~EditorDialog() {
    qDebug() << "dialog is safely deleted~";
}

void EditorDialog::initUI() {
    this->setWindowTitle("ItemEditDialog");
    this->setMinimumSize(210, 300);
    this->setAttribute(Qt::WA_DeleteOnClose);

    std::string item_type = itemInfo_[0].toStdString();
    auto index = distance(type_name_list.begin(), std::find(type_name_list.begin(), type_name_list.end(), item_type));
    if (index >= type_name_list.size()) return;
    type_ = index;

    auto layout = new QGridLayout;
    this->setLayout(layout);

    //type region
    auto button_group = new QButtonGroup(this);
    auto type_label = new QLabel("Type:", this);
    layout->addWidget(type_label, 0, 0, 1, 1);
    for (int i = 0; i < type_name_list.size(); ++i) {
        QString button_name = QString::fromStdString(type_name_list[i]);
        auto button = new QRadioButton(button_name, this);
        button->setIcon(QIcon("../../resources/" + button_name.toLower() + ".png"));

        button_group->addButton(button);
        button_group->setId(button, i);
        layout->addWidget(button, i, 1, 1, 1);
        if (i == index) button->setChecked(true);
    }
    //Notice:connect behind button initial
    connect(button_group, SIGNAL(buttonToggled(int,bool)), this, SLOT(buttonsToggled(int,bool)));

    //fields region
    const auto& fields_name = fields_vec[index];
    for (int j = 0; j < fields_name.size(); ++j) {
        QString field_name = QString::fromStdString(fields_name[j]);
        QString label_object_name = "label" + QString::number(j);
        QString text_object_name = "text" + QString::number(j);

        auto label = new QLabel(field_name, this);
        label->setObjectName(label_object_name);
        auto text = new QLineEdit(this);
        text->setObjectName(text_object_name);
        text->setFixedWidth(90);

        layout->addWidget(label, type_name_list.size() + j, 0, 1, 1);
        layout->addWidget(text, type_name_list.size() + j, 1, 1, 1);
    }

    //button region
    auto button_layout = new QHBoxLayout;
    {
        auto save_button = new QPushButton("save", this);
        save_button->setObjectName("save_button");
        save_button->setFixedWidth(85);
        connect(save_button, SIGNAL(clicked()), this, SLOT(save()));
        layout->addWidget(save_button, type_name_list.size() + fields_name.size(), 0, 1, 1);

        auto cancel_button = new QPushButton("cancel", this);
        cancel_button->setAutoDefault(true);
        cancel_button->setFixedWidth(85);
        connect(cancel_button, SIGNAL(clicked()), this, SLOT(close()));
        layout->addWidget(cancel_button, type_name_list.size() + fields_name.size(), 1, 1, 1);
    }
}

void EditorDialog::save() {

    QStringList textInfo;
    textInfo.append(QString::fromStdString(type_name_list[type_]));
    for (int i = 0; i < fields_vec[type_].size(); ++i) {
        QString objectName = "text" + QString::number(i);
        auto text = this->findChild<QLineEdit*>(objectName);

        textInfo.append(text->text());
    }

    emit postItemInfo(textInfo);
    this->close();
}

void EditorDialog::buttonsToggled(int index, bool checked) {
    if (checked) {
        const auto& fields_name = fields_vec[index];
        for (int j = 0; j < fields_name.size(); ++j) {
            QString field_name = QString::fromStdString(fields_name[j]);
            QString label_object_name = "label" + QString::number(j);
            auto label = this->findChild<QLabel*>(label_object_name);
            label->setText(field_name);
        }
        type_ = index;
    }
}
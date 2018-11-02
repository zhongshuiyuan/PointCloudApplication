//
// Created by WuKun on 9/4/18.
// Contact me:wk707060335@gmail.com
//

#include <string>
#include <vector>
#include <iostream>
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

const QStringList type_name_list = { "Uncertain", "RoadEdge", "CrossWalk", "StopLine", "Lane" };

const QStringList Uncertain_fields = { "", "", "" };
const QStringList RoadEdge_fields = { "", "", "" };
const QStringList CrossWalk_fields = { "Type", "BdID", "" };
const QStringList StopLine_fields = { "TLID", "SignID", "" };
const QStringList Lane_fields = { "LCnt", "Lno", "" };

const QVector<QStringList> fields_vec = { Uncertain_fields, RoadEdge_fields, CrossWalk_fields, StopLine_fields, Lane_fields };

EditorDialog::EditorDialog(QStringList& itemInfo, QWidget *parent) :
    QDialog(parent),
    type_(0),
    itemInfo_(itemInfo){
    initUI();
}

EditorDialog::~EditorDialog() {
    //qDebug() << "dialog is safely deleted~";
}

void EditorDialog::initUI() {

    this->setWindowTitle("ItemEditDialog");
    this->setMaximumSize(210, 300);
    this->setAttribute(Qt::WA_DeleteOnClose);

    QString item_type = itemInfo_[0];
    auto index = type_name_list.indexOf(item_type);
    if (index >= type_name_list.size()) return;
    type_ = index;

    auto layout = new QGridLayout;
    this->setLayout(layout);

    //type region
    auto button_group = new QButtonGroup(this);
    auto type_label = new QLabel("Type:", this);
    layout->addWidget(type_label, 0, 0, 1, 1);
    for (int i = 0; i < type_name_list.size(); ++i) {
        QString button_name = type_name_list[i];
        auto button = new QRadioButton(button_name, this);
        button->setIcon(QIcon("../../resources/" + button_name.toLower() + ".png"));
        button->setVisible(item_type == "Lane" ? i == index : i != type_name_list.indexOf("Lane"));

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
        QString field_name = fields_name[j];
        QString label_object_name = "label" + QString::number(j);
        QString text_object_name = "text" + QString::number(j);

        auto label = new QLabel(field_name, this);
        label->setObjectName(label_object_name);
        auto text = new QLineEdit(this);
        text->setText(itemInfo_[j + 1]);
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

        auto cancel_button = new QPushButton("delete", this);
        cancel_button->setAutoDefault(true);
        cancel_button->setFixedWidth(85);
        connect(cancel_button, SIGNAL(clicked()), this, SLOT(deleteItem()));
        layout->addWidget(cancel_button, type_name_list.size() + fields_name.size(), 1, 1, 1);
    }
}

void EditorDialog::buttonsToggled(int index, bool checked) {
    if (checked) {
        const auto& fields_name = fields_vec[index];
        for (int j = 0; j < fields_name.size(); ++j) {
            QString field_name = fields_name[j];
            QString label_object_name = "label" + QString::number(j);
            auto label = this->findChild<QLabel*>(label_object_name);
            label->setText(field_name);
        }
        type_ = index;
    }
}

void EditorDialog::save() {
    QStringList textInfo;
    textInfo.append(type_name_list[type_]);

    for (int i = 0; i < fields_vec[type_].size(); ++i) {
        QString objectName = "text" + QString::number(i);
        auto text = this->findChild<QLineEdit*>(objectName);

        textInfo.append(text->text());
    }
    qDebug() << "textInfo:" << textInfo;
    emit postItemInfo(textInfo);
    this->close();
}

void EditorDialog::deleteItem() {
    QString itemType = type_name_list[type_];
    qDebug() << "itemType:" << itemType;

    emit deleteItemInfo(itemType);
    this->close();
}

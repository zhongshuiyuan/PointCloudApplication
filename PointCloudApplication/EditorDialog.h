//
// Created by WuKun on 9/4/18.
// Contact me:wk707060335@gmail.com
//

#ifndef POINTCLOUDAPPLICATION_EDITORDIALOG_H
#define POINTCLOUDAPPLICATION_EDITORDIALOG_H

#include <QtWidgets/QWidget>
#include <QtWidgets/QDialog>
#include <QtWidgets/QLabel>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QRadioButton>
#include <QtCore/QStringList>

class EditorDialog : public QDialog {
    Q_OBJECT
public:
    explicit EditorDialog(QStringList& itemInfo, QWidget* parent = nullptr);
    ~EditorDialog() final;

    Q_DISABLE_COPY(EditorDialog);

private:
    void initUI();

    int type_;
    QStringList itemInfo_;

Q_SIGNALS:
    void postItemInfo(QStringList itemInfo);

public Q_SLOTS:
    void save();
    void buttonsToggled(int index, bool checked);
};


#endif //POINTCLOUDAPPLICATION_EDITORWIDGET_H

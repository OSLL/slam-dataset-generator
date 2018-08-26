#ifndef PROJECT_FILE_H
#define PROJECT_FILE_H

#include <QSettings>
#include <QTemporaryDir>

#include <QPixmap>
#include <QHash>

class ProjectFile {
public:
    enum ErrorCode { NoError, Corrupted, IOError };

    ProjectFile();
    ~ProjectFile();

    QSettings *settings();

    bool addPixmap(const QString &id, const QPixmap &pix) const;
    QPixmap getPixmap(const QString &id) const;

    ErrorCode save(const QString &fileName);
    ErrorCode load(const QString &fileName);

private:
    enum LoadMode { Header, Name, Data };

    QTemporaryDir _dir;
    QSettings *_settings;
};

#endif // PROJECT_FILE_H

#include "project_file.h"

#include <QDir>
#include <QSaveFile>
#include <QTemporaryFile>

// TODO: POC file format, should be improved

ProjectFile::ProjectFile() : _settings(0) {}

ProjectFile::~ProjectFile() {
    delete _settings;
}

QSettings *ProjectFile::settings() {
    if(!_settings) {
        if(_dir.isValid()) {
            _settings = new QSettings(_dir.path() + "/config.ini", QSettings::IniFormat);
        } else {
            return 0;
        }
    }
    return _settings;
}

bool ProjectFile::addPixmap(const QString &id, const QPixmap &pix) const {
    return pix.save(QString("%1/%2.png").arg(_dir.path()).arg(id), "png");
}

QPixmap ProjectFile::getPixmap(const QString &id) const {
    if(id.isEmpty()) return QPixmap();

    auto fileName = QString("%1/%2.png").arg(_dir.path()).arg(id);
    return QPixmap(fileName, "png");
}

ProjectFile::ErrorCode ProjectFile::save(const QString &fileName) {
    QSaveFile outFile(fileName);
    if(!outFile.open(QFile::WriteOnly)) return IOError;

    _settings->sync();

    for(auto i : QDir(_dir.path()).entryInfoList(QDir::Files)) {
        auto name = i.fileName().toUtf8();
        qint32 nsize = name.size();

        QByteArray data;
        qint64 dsize;

        if(i.suffix() == "ini") {
            QFile file(i.absoluteFilePath());
            if(!file.open(QFile::ReadOnly)) return IOError;

            data = qCompress(file.readAll());
            dsize = data.size();
        } else {
            QFile file(i.absoluteFilePath());
            if(!file.open(QFile::ReadOnly)) return IOError;

            dsize = i.size();
            data = file.readAll();
        }

        if(outFile.write(reinterpret_cast<char*>(&nsize), sizeof(nsize)) < 0) return IOError;
        if(outFile.write(reinterpret_cast<char*>(&dsize), sizeof(dsize)) < 0) return IOError;
        if(outFile.write(name) < 0) return IOError;
        if(outFile.write(data) < 0) return IOError;
    }

    return outFile.commit() ? NoError : IOError;
}

ProjectFile::ErrorCode ProjectFile::load(const QString &fileName) {
    QFile inFile(fileName);
    if(!inFile.open(QFile::ReadOnly)) return IOError;

    QString name;
    qint32 nsize = 0;
    qint64 dsize = 0;
    static const auto hsize = sizeof(nsize) + sizeof(dsize);
    static const qint64 bufSize = 32 * 1024;

    char buf[bufSize];
    auto mode = Header;

    while(!inFile.atEnd()) {
        switch(mode) {
        case Header:
            if(inFile.read(buf, hsize) != hsize) {
                return Corrupted;
            }
            nsize = *reinterpret_cast<qint32*>(buf);
            dsize = *reinterpret_cast<qint64*>(buf + sizeof(nsize));
            mode = Name;
            break;
        case Name:
            if(nsize > bufSize) return Corrupted;
            if(inFile.read(buf, nsize) != nsize) return IOError;
            name = QString::fromUtf8(buf, nsize);
            mode = Data;
            break;
        case Data: {
            QTemporaryFile tmp;
            if(!tmp.open()) return IOError;

            while(dsize != 0) {
                auto bytesRead = inFile.read(buf, qMin(dsize, bufSize));
                if(bytesRead < 0) return IOError;

                if(tmp.write(buf, bytesRead) != bytesRead) {
                    return IOError;
                }

                dsize -= bytesRead;
            }

            tmp.close();

            if(name.endsWith(".ini")) {
                if(!tmp.open()) return IOError;

                QFile file(_dir.path() + "/" + name);
                if(!file.open(QFile::WriteOnly)) return IOError;
                if(file.write(qUncompress(tmp.readAll())) < 0) return IOError;
            } else {
                if(!QFile::copy(tmp.fileName(), _dir.path() + "/" + name)) return IOError;
            }
            mode = Header;
            break;
        }
        default:
            return Corrupted;
        }
    }

    if(mode != Header) return Corrupted;
    if(!QFile::exists(_dir.path() + "/config.ini")) return Corrupted;
    return NoError;
}

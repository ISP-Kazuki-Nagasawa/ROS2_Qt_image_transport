#ifndef VIDEO_PROVIDER_H
#define VIDEO_PROVIDER_H

#include <QObject>
#include <QQuickImageProvider>
#include <QPixmap>

class VideoProvider : public QObject, public QQuickImageProvider
{
    Q_OBJECT
public:
    explicit VideoProvider(QObject *parent = nullptr);
    QPixmap requestPixmap(const QString &id, QSize *size, const QSize &requestedSize);

public slots:
    void OnReceiveImage(QImage image);

private:
    QImage image_;
};

#endif // VIDEO_PROVIDER_H

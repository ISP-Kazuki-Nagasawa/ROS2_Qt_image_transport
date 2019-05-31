#include "video_provider.h"

VideoProvider::VideoProvider(QObject *parent)
    : QObject(parent), QQuickImageProvider(QQuickImageProvider::Pixmap)
{
}


QPixmap VideoProvider::requestPixmap(const QString &id, QSize *size, const QSize &requestedSize)
{
    if (!this->image_.isNull()) {
        QPixmap pixmap = QPixmap::fromImage(this->image_);
        *size = QSize(this->image_.width(), this->image_.height());
        int width  = requestedSize.width()  > 0 ? requestedSize.width()  : this->image_.width();
        int height = requestedSize.height() > 0 ? requestedSize.height() : this->image_.height();
        pixmap.scaled(width, height, Qt::KeepAspectRatio, Qt::FastTransformation);
        return pixmap;
    }
    else {
        QPixmap pixmap(requestedSize.width() > 0 ? requestedSize.width() : 640, requestedSize.height() > 0 ? requestedSize.height() : 480);
        *size = QSize(pixmap.width(), pixmap.height());
        pixmap.fill(QColor("green").rgba());
        return pixmap;
    }
}


void VideoProvider::OnReceiveImage(QImage image)
{
    this->image_ = image;
}

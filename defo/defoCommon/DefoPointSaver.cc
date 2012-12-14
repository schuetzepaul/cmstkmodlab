#include "DefoPointSaver.h"

const QString PointSaver::LINE_FORMAT = "%1\t%2\t%3\t%4\t%5\t%6\t%7\t%8\n";

PointSaver::PointSaver(const QString &filename, QObject* parent) :
  QFile(filename, parent)
{
  open( QIODevice::WriteOnly | QIODevice::Truncate );
}

PointSaver::~PointSaver() {
  close();
}

void PointSaver::writePoints(const DefoPointCollection &points)
{
    for (DefoPointCollection::const_iterator it = points.begin();
         it != points.end();
         ++it) {
         writePoint(*it);
    }
}

void PointSaver::writePoint(const DefoPoint& point)
{
  double x = point.getX();
  double y = point.getY();
  QColor color = point.getColor();
  float hue = color.hsvHueF();
  float saturation = color.hsvSaturationF();
  float value = color.valueF();

  QString line = LINE_FORMAT
      .arg(x, 0, 'e', 6)
      .arg(y, 0, 'e', 6)
      .arg(hue, 0, 'e', 6)
      .arg(saturation, 0, 'e', 6)
      .arg(value, 0, 'e', 6)
      .arg((int)(point.isIndexed()))
      .arg(point.getIndex().first)
      .arg(point.getIndex().second);
  
  write(line.toAscii());
}

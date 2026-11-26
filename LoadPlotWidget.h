#pragma once

#include <QColor>
#include <QString>
#include <QStringList>
#include <QVector>
#include <QWidget>

#include <array>
#include <deque>

// Scrolling strip chart of the wheel load ratios (load / static load) over the last few seconds.
// Hazard events are drawn as coloured bands behind the curves.
class LoadPlotWidget : public QWidget
{
    Q_OBJECT

public:
    static constexpr int kMaxSeries = 8;

    explicit LoadPlotWidget(QWidget* parent = nullptr);

    void setSeries(const QStringList& names, const QVector<QColor>& colors);
    void setWindowSeconds(double seconds) { m_window = seconds; }

    void addSample(double t, const double* values);
    void setOpenSpan(bool open, double t0, const QString& label);
    void addSpan(double t0, double t1, const QString& label, const QColor& color);
    void clear();

    QSize sizeHint() const override { return QSize(480, 220); }
    QSize minimumSizeHint() const override { return QSize(320, 150); }

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    struct Sample
    {
        double t = 0.0;
        std::array<float, kMaxSeries> v{};
    };
    struct Span
    {
        double t0 = 0.0;
        double t1 = 0.0;
        QString label;
        QColor color;
    };

    std::deque<Sample> m_samples;
    std::deque<Span> m_spans;
    QStringList m_names;
    QVector<QColor> m_colors;
    int m_count = 0;

    double m_window = 8.0;
    double m_tNow = 0.0;

    bool m_openSpan = false;
    double m_openT0 = 0.0;
    QString m_openLabel;
};

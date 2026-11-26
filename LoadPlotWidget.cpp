#include "LoadPlotWidget.h"

#include <QPainter>
#include <QPainterPath>
#include <QPolygonF>

#include <algorithm>
#include <cmath>
#include <limits>

LoadPlotWidget::LoadPlotWidget(QWidget* parent)
    : QWidget(parent)
{
    setAutoFillBackground(false);
}

void LoadPlotWidget::setSeries(const QStringList& names, const QVector<QColor>& colors)
{
    m_names = names;
    m_colors = colors;
    m_count = std::min<int>(kMaxSeries, std::min<int>(names.size(), colors.size()));
}

void LoadPlotWidget::addSample(double t, const double* values)
{
    Sample s;
    s.t = t;
    for (int k = 0; k < m_count; ++k)
        s.v[k] = static_cast<float>(values[k]);
    m_samples.push_back(s);
    m_tNow = t;

    const double keep = m_tNow - 1.5 * m_window;
    while (!m_samples.empty() && m_samples.front().t < keep)
        m_samples.pop_front();
    while (!m_spans.empty() && m_spans.front().t1 < keep)
        m_spans.pop_front();
}

void LoadPlotWidget::setOpenSpan(bool open, double t0, const QString& label)
{
    m_openSpan = open;
    m_openT0 = t0;
    m_openLabel = label;
}

void LoadPlotWidget::addSpan(double t0, double t1, const QString& label, const QColor& color)
{
    m_spans.push_back({ t0, t1, label, color });
}

void LoadPlotWidget::clear()
{
    m_samples.clear();
    m_spans.clear();
    m_openSpan = false;
    m_tNow = 0.0;
    update();
}

void LoadPlotWidget::paintEvent(QPaintEvent*)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.fillRect(rect(), QColor(24, 28, 34));

    const double left = 48.0, right = 12.0, top = 24.0, bottom = 26.0;
    const QRectF plot(left, top, width() - left - right, height() - top - bottom);
    if (plot.width() < 40.0 || plot.height() < 40.0)
        return;

    const double t1 = std::max(m_tNow, m_window);
    const double t0 = t1 - m_window;

    // y range: at least 0..3, grows with the data
    double yMax = 3.0;
    for (const auto& s : m_samples)
    {
        if (s.t < t0)
            continue;
        for (int k = 0; k < m_count; ++k)
            yMax = std::max(yMax, static_cast<double>(s.v[k]));
    }
    yMax = std::ceil(yMax * 2.0 + 0.2) / 2.0;

    auto X = [&](double t) { return plot.left() + (t - t0) / m_window * plot.width(); };
    auto Y = [&](double v) { return plot.bottom() - v / yMax * plot.height(); };

    // event bands
    QFont small = font();
    small.setPointSizeF(small.pointSizeF() * 0.9);
    p.setFont(small);
    auto drawBand = [&](double a, double b, const QString& label, QColor c)
    {
        if (b < t0)
            return;
        a = std::max(a, t0);
        const QRectF r(X(a), plot.top(), std::max(2.0, X(b) - X(a)), plot.height());
        QColor fill = c;
        fill.setAlpha(55);
        p.fillRect(r, fill);
        p.setPen(c);
        p.drawText(QPointF(r.left() + 3.0, plot.top() + 12.0), label);
    };
    for (const auto& sp : m_spans)
        drawBand(sp.t0, sp.t1, sp.label, sp.color);
    if (m_openSpan)
        drawBand(m_openT0, t1, m_openLabel, QColor(220, 220, 220));

    // grid + y labels
    for (double v = 0.0; v <= yMax + 1e-9; v += 0.5)
    {
        const bool major = std::fmod(v, 1.0) < 1e-9;
        p.setPen(QPen(QColor(70, 78, 90), 1.0, major ? Qt::SolidLine : Qt::DotLine));
        p.drawLine(QPointF(plot.left(), Y(v)), QPointF(plot.right(), Y(v)));
        if (major)
        {
            p.setPen(QColor(170, 176, 186));
            p.drawText(QRectF(0.0, Y(v) - 8.0, left - 6.0, 16.0), Qt::AlignRight | Qt::AlignVCenter,
                       QString::number(v, 'f', 1));
        }
    }

    // static load reference
    p.setPen(QPen(QColor(230, 230, 230, 170), 1.2, Qt::DashLine));
    p.drawLine(QPointF(plot.left(), Y(1.0)), QPointF(plot.right(), Y(1.0)));

    // time axis
    p.setPen(QColor(170, 176, 186));
    for (int k = 0; k <= static_cast<int>(m_window); k += 2)
    {
        const double t = t1 - k;
        const double x = X(t);
        p.drawLine(QPointF(x, plot.bottom()), QPointF(x, plot.bottom() + 4.0));
        p.drawText(QRectF(x - 24.0, plot.bottom() + 4.0, 48.0, 16.0), Qt::AlignCenter,
                   k == 0 ? QString("now") : QString("-%1 s").arg(k));
    }

    // curves: min/max per pixel column so that short impact peaks are never lost
    constexpr int kNoColumn = std::numeric_limits<int>::min();
    p.setClipRect(plot.adjusted(-1, -1, 1, 1));
    p.setRenderHint(QPainter::Antialiasing, false);  // thousands of points per frame: keep it cheap
    for (int k = 0; k < m_count; ++k)
    {
        QPolygonF poly;
        int column = kNoColumn;
        float mn = 0.0f, mx = 0.0f;
        auto flush = [&]()
        {
            if (column == kNoColumn)
                return;
            poly << QPointF(column, Y(mx));
            if (mn != mx)
                poly << QPointF(column, Y(mn));
        };
        for (const auto& s : m_samples)
        {
            if (s.t < t0)
                continue;
            const int c = static_cast<int>(X(s.t));
            if (c != column)
            {
                flush();
                column = c;
                mn = mx = s.v[k];
            }
            else
            {
                mn = std::min(mn, s.v[k]);
                mx = std::max(mx, s.v[k]);
            }
        }
        flush();
        p.setPen(QPen(m_colors[k], 1.4));
        p.drawPolyline(poly);
    }
    p.setClipping(false);
    p.setRenderHint(QPainter::Antialiasing, true);

    // title + legend
    p.setPen(QColor(220, 224, 230));
    p.drawText(QPointF(plot.left(), 15.0), "Load / static");
    double lx = plot.left() + 100.0;
    for (int k = 0; k < m_count; ++k)
    {
        p.setPen(QPen(m_colors[k], 3.0));
        p.drawLine(QPointF(lx, 11.0), QPointF(lx + 12.0, 11.0));
        p.setPen(QColor(210, 214, 220));
        p.drawText(QPointF(lx + 15.0, 15.0), m_names[k]);
        lx += 52.0;
    }
}

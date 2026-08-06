package com.xreal.parallax;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.LinearGradient;
import android.graphics.Paint;
import android.graphics.RectF;
import android.graphics.Shader;
import android.util.AttributeSet;
import android.view.View;

public final class ParallaxView extends View {
    private final Paint backgroundPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint gridPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint panelPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint panelStrokePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint textPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final RectF panelRect = new RectF();

    private float yaw;
    private float pitch;
    private String status = "Phone or DeX display";

    public ParallaxView(Context context) {
        super(context);
        init();
    }

    public ParallaxView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    public void setHeadPose(float yaw, float pitch) {
        this.yaw = yaw;
        this.pitch = pitch;
        invalidate();
    }

    public void setStatus(String status) {
        this.status = status;
        invalidate();
    }

    private void init() {
        setKeepScreenOn(true);
        backgroundPaint.setColor(Color.rgb(9, 11, 16));
        gridPaint.setColor(Color.argb(55, 120, 220, 232));
        gridPaint.setStrokeWidth(1.5f);
        panelStrokePaint.setStyle(Paint.Style.STROKE);
        panelStrokePaint.setStrokeWidth(3.0f);
        panelStrokePaint.setColor(Color.argb(210, 230, 244, 246));
        textPaint.setColor(Color.rgb(236, 242, 244));
        textPaint.setTextAlign(Paint.Align.CENTER);
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);

        int width = getWidth();
        int height = getHeight();
        canvas.drawRect(0, 0, width, height, backgroundPaint);

        drawGrid(canvas, width, height);
        drawPanels(canvas, width, height);
        drawStatus(canvas, width, height);
        postInvalidateOnAnimation();
    }

    private void drawGrid(Canvas canvas, int width, int height) {
        float horizon = height * 0.58f + pitch * height * 0.18f;
        float spacing = Math.max(48.0f, width / 26.0f);

        for (float x = width * 0.5f; x < width; x += spacing) {
            canvas.drawLine(width * 0.5f, horizon, x, height, gridPaint);
            canvas.drawLine(width * 0.5f, horizon, width - x, height, gridPaint);
        }

        for (int i = 0; i < 9; i++) {
            float y = horizon + (height - horizon) * i / 8.0f;
            canvas.drawLine(0, y, width, y, gridPaint);
        }
    }

    private void drawPanels(Canvas canvas, int width, int height) {
        float centerX = width * 0.5f - yaw * width * 0.22f;
        float centerY = height * 0.45f + pitch * height * 0.22f;
        float panelWidth = width * 0.25f;
        float panelHeight = panelWidth * 9.0f / 16.0f;
        float gap = width * 0.035f;

        int[] colors = {
            Color.rgb(46, 112, 210),
            Color.rgb(210, 83, 60),
            Color.rgb(52, 152, 96),
            Color.rgb(208, 154, 70)
        };

        for (int i = 0; i < 4; i++) {
            float offset = i - 1.5f;
            float arc = Math.abs(offset) * 0.09f;
            float x = centerX + offset * (panelWidth + gap);
            float y = centerY + arc * height;
            float scale = 1.0f - Math.abs(offset) * 0.08f;
            float w = panelWidth * scale;
            float h = panelHeight * scale;

            panelRect.set(x - w / 2.0f, y - h / 2.0f, x + w / 2.0f, y + h / 2.0f);
            panelPaint.setShader(new LinearGradient(
                    panelRect.left,
                    panelRect.top,
                    panelRect.right,
                    panelRect.bottom,
                    lighten(colors[i]),
                    colors[i],
                    Shader.TileMode.CLAMP));
            canvas.drawRoundRect(panelRect, 18.0f, 18.0f, panelPaint);
            panelPaint.setShader(null);
            canvas.drawRoundRect(panelRect, 18.0f, 18.0f, panelStrokePaint);

            textPaint.setTextSize(Math.max(22.0f, width / 80.0f));
            canvas.drawText("Panel " + (i + 1), x, y + textPaint.getTextSize() * 0.3f, textPaint);
        }
    }

    private void drawStatus(Canvas canvas, int width, int height) {
        textPaint.setTextSize(Math.max(18.0f, width / 100.0f));
        canvas.drawText("XREAL Parallax - " + status, width * 0.5f, height * 0.08f, textPaint);
    }

    private int lighten(int color) {
        int r = Math.min(255, (int) (Color.red(color) * 1.25f));
        int g = Math.min(255, (int) (Color.green(color) * 1.25f));
        int b = Math.min(255, (int) (Color.blue(color) * 1.25f));
        return Color.rgb(r, g, b);
    }
}

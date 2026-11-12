#ifndef GUI_WINDOW_HPP
#define GUI_WINDOW_HPP

/**
 * @file gui_window.hpp
 * @brief Lightweight, header-only 2D software renderer and X11 window wrapper.
 *
 * Dependencies: Standard C++17 + system libX11 (-lX11), present on virtually every
 * Linux desktop system (works natively on X11 and via XWayland on Wayland).
 * Features:
 *  - 32-bit ARGB offscreen pixel buffer (flicker-free double buffering)
 *  - Anti-aliased / thick line drawing, filled/outlined circles, rectangles
 *  - Embedded 8x8 bitmap font (ASCII 32..126) for HUD text without font libraries
 *  - Non-blocking keyboard & mouse polling (interactive target control, pause, reset)
 */

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/keysym.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace gui {

struct Color {
    uint8_t r, g, b;
    constexpr uint32_t to_u32() const {
        return (0xFFu << 24) | (static_cast<uint32_t>(r) << 16) |
               (static_cast<uint32_t>(g) << 8) | static_cast<uint32_t>(b);
    }
};

namespace Palette {
    constexpr Color Background {18, 22, 31};
    constexpr Color Grid       {35, 43, 58};
    constexpr Color Axis       {70, 84, 112};
    constexpr Color PanelBg    {27, 34, 48};
    constexpr Color PanelBorder{62, 74, 99};
    constexpr Color White      {240, 244, 248};
    constexpr Color Gray       {145, 155, 175};
    constexpr Color Green      {46, 204, 113};
    constexpr Color Blue       {52, 152, 219};
    constexpr Color Cyan       {26, 188, 156};
    constexpr Color Red        {231, 76, 60};
    constexpr Color Orange     {243, 156, 18};
    constexpr Color Yellow     {241, 196, 15};
    constexpr Color Magenta    {155, 89, 182};
}

// Compact 8x8 ASCII bitmap font (characters 32 to 126)
inline const uint8_t* font8x8_basic(char c) {
    static const uint8_t font[95][8] = {
        {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // ' '
        {0x18,0x3C,0x3C,0x18,0x18,0x00,0x18,0x00}, // '!'
        {0x36,0x36,0x00,0x00,0x00,0x00,0x00,0x00}, // '"'
        {0x36,0x36,0x7F,0x36,0x7F,0x36,0x36,0x00}, // '#'
        {0x0C,0x3E,0x03,0x1E,0x30,0x1F,0x0C,0x00}, // '$'
        {0x00,0x63,0x33,0x18,0x0C,0x66,0x63,0x00}, // '%'
        {0x1C,0x36,0x1C,0x6E,0x3B,0x33,0x6E,0x00}, // '&'
        {0x06,0x06,0x03,0x00,0x00,0x00,0x00,0x00}, // '''
        {0x18,0x0C,0x06,0x06,0x06,0x0C,0x18,0x00}, // '('
        {0x06,0x0C,0x18,0x18,0x18,0x0C,0x06,0x00}, // ')'
        {0x00,0x66,0x3C,0xFF,0x3C,0x66,0x00,0x00}, // '*'
        {0x00,0x0C,0x0C,0x3F,0x0C,0x0C,0x00,0x00}, // '+'
        {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C,0x06}, // ','
        {0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x00}, // '-'
        {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C,0x00}, // '.'
        {0x60,0x30,0x18,0x0C,0x06,0x03,0x01,0x00}, // '/'
        {0x3E,0x63,0x73,0x7B,0x6F,0x67,0x3E,0x00}, // '0'
        {0x0C,0x0E,0x0C,0x0C,0x0C,0x0C,0x3F,0x00}, // '1'
        {0x1E,0x33,0x30,0x1C,0x06,0x33,0x3F,0x00}, // '2'
        {0x1E,0x33,0x30,0x1C,0x30,0x33,0x1E,0x00}, // '3'
        {0x38,0x3C,0x36,0x33,0x7F,0x30,0x78,0x00}, // '4'
        {0x3F,0x03,0x1F,0x30,0x30,0x33,0x1E,0x00}, // '5'
        {0x1C,0x06,0x03,0x1F,0x33,0x33,0x1E,0x00}, // '6'
        {0x3F,0x33,0x30,0x18,0x0C,0x0C,0x0C,0x00}, // '7'
        {0x1E,0x33,0x33,0x1E,0x33,0x33,0x1E,0x00}, // '8'
        {0x1E,0x33,0x33,0x3E,0x30,0x18,0x0E,0x00}, // '9'
        {0x00,0x0C,0x0C,0x00,0x00,0x0C,0x0C,0x00}, // ':'
        {0x00,0x0C,0x0C,0x00,0x00,0x0C,0x0C,0x06}, // ';'
        {0x18,0x0C,0x06,0x03,0x06,0x0C,0x18,0x00}, // '<'
        {0x00,0x00,0x3F,0x00,0x00,0x3F,0x00,0x00}, // '='
        {0x06,0x0C,0x18,0x30,0x18,0x0C,0x06,0x00}, // '>'
        {0x1E,0x33,0x30,0x18,0x0C,0x00,0x0C,0x00}, // '?'
        {0x3E,0x63,0x7B,0x7B,0x7B,0x03,0x1E,0x00}, // '@'
        {0x0C,0x1E,0x33,0x33,0x3F,0x33,0x33,0x00}, // 'A'
        {0x3F,0x66,0x66,0x3E,0x66,0x66,0x3F,0x00}, // 'B'
        {0x3C,0x66,0x03,0x03,0x03,0x66,0x3C,0x00}, // 'C'
        {0x1F,0x36,0x66,0x66,0x66,0x36,0x1F,0x00}, // 'D'
        {0x7F,0x46,0x16,0x1E,0x16,0x46,0x7F,0x00}, // 'E'
        {0x7F,0x46,0x16,0x1E,0x16,0x06,0x0F,0x00}, // 'F'
        {0x3C,0x66,0x03,0x03,0x73,0x66,0x7C,0x00}, // 'G'
        {0x33,0x33,0x33,0x3F,0x33,0x33,0x33,0x00}, // 'H'
        {0x1E,0x0C,0x0C,0x0C,0x0C,0x0C,0x1E,0x00}, // 'I'
        {0x78,0x30,0x30,0x30,0x33,0x33,0x1E,0x00}, // 'J'
        {0x67,0x66,0x36,0x1E,0x36,0x66,0x67,0x00}, // 'K'
        {0x0F,0x06,0x06,0x06,0x46,0x66,0x7F,0x00}, // 'L'
        {0x63,0x77,0x7F,0x7F,0x6B,0x63,0x63,0x00}, // 'M'
        {0x63,0x67,0x6F,0x7B,0x73,0x63,0x63,0x00}, // 'N'
        {0x1C,0x36,0x63,0x63,0x63,0x36,0x1C,0x00}, // 'O'
        {0x3F,0x66,0x66,0x3E,0x06,0x06,0x0F,0x00}, // 'P'
        {0x1E,0x33,0x33,0x33,0x3B,0x1E,0x38,0x00}, // 'Q'
        {0x3F,0x66,0x66,0x3E,0x36,0x66,0x67,0x00}, // 'R'
        {0x1E,0x33,0x07,0x0E,0x38,0x33,0x1E,0x00}, // 'S'
        {0x3F,0x2D,0x0C,0x0C,0x0C,0x0C,0x1E,0x00}, // 'T'
        {0x33,0x33,0x33,0x33,0x33,0x33,0x3F,0x00}, // 'U'
        {0x33,0x33,0x33,0x33,0x33,0x1E,0x0C,0x00}, // 'V'
        {0x63,0x63,0x63,0x6B,0x7F,0x77,0x63,0x00}, // 'W'
        {0x63,0x63,0x36,0x1C,0x1C,0x36,0x63,0x00}, // 'X'
        {0x33,0x33,0x33,0x1E,0x0C,0x0C,0x1E,0x00}, // 'Y'
        {0x7F,0x63,0x31,0x18,0x4C,0x66,0x7F,0x00}, // 'Z'
        {0x1E,0x06,0x06,0x06,0x06,0x06,0x1E,0x00}, // '['
        {0x03,0x06,0x0C,0x18,0x30,0x60,0x40,0x00}, // '\'
        {0x1E,0x18,0x18,0x18,0x18,0x18,0x1E,0x00}, // ']'
        {0x08,0x1C,0x36,0x63,0x00,0x00,0x00,0x00}, // '^'
        {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF}, // '_'
        {0x0C,0x0C,0x18,0x00,0x00,0x00,0x00,0x00}, // '`'
        {0x00,0x00,0x1E,0x30,0x3E,0x33,0x6E,0x00}, // 'a'
        {0x07,0x06,0x06,0x3E,0x66,0x66,0x3B,0x00}, // 'b'
        {0x00,0x00,0x1E,0x33,0x03,0x33,0x1E,0x00}, // 'c'
        {0x38,0x30,0x30,0x3e,0x33,0x33,0x6E,0x00}, // 'd'
        {0x00,0x00,0x1E,0x33,0x3f,0x03,0x1E,0x00}, // 'e'
        {0x1C,0x36,0x06,0x0f,0x06,0x06,0x0F,0x00}, // 'f'
        {0x00,0x00,0x6E,0x33,0x33,0x3E,0x30,0x1F}, // 'g'
        {0x07,0x06,0x36,0x6E,0x66,0x66,0x67,0x00}, // 'h'
        {0x0C,0x00,0x0E,0x0C,0x0C,0x0C,0x1E,0x00}, // 'i'
        {0x30,0x00,0x30,0x30,0x30,0x33,0x33,0x1E}, // 'j'
        {0x07,0x06,0x66,0x36,0x1E,0x36,0x67,0x00}, // 'k'
        {0x0E,0x0C,0x0C,0x0C,0x0C,0x0C,0x1E,0x00}, // 'l'
        {0x00,0x00,0x33,0x7F,0x7F,0x6B,0x63,0x00}, // 'm'
        {0x00,0x00,0x1F,0x33,0x33,0x33,0x33,0x00}, // 'n'
        {0x00,0x00,0x1E,0x33,0x33,0x33,0x1E,0x00}, // 'o'
        {0x00,0x00,0x3B,0x66,0x66,0x3E,0x06,0x0F}, // 'p'
        {0x00,0x00,0x6E,0x33,0x33,0x3E,0x30,0x78}, // 'q'
        {0x00,0x00,0x3B,0x6E,0x66,0x06,0x0F,0x00}, // 'r'
        {0x00,0x00,0x3E,0x03,0x1E,0x30,0x1F,0x00}, // 's'
        {0x08,0x0C,0x3E,0x0C,0x0C,0x2C,0x18,0x00}, // 't'
        {0x00,0x00,0x33,0x33,0x33,0x33,0x6E,0x00}, // 'u'
        {0x00,0x00,0x33,0x33,0x33,0x1E,0x0C,0x00}, // 'v'
        {0x00,0x00,0x63,0x6B,0x7F,0x7F,0x36,0x00}, // 'w'
        {0x00,0x00,0x63,0x36,0x1C,0x36,0x63,0x00}, // 'x'
        {0x00,0x00,0x33,0x33,0x33,0x3E,0x30,0x1F}, // 'y'
        {0x00,0x00,0x3F,0x19,0x0C,0x26,0x3F,0x00}, // 'z'
        {0x38,0x0C,0x0C,0x07,0x0C,0x0C,0x38,0x00}, // '{'
        {0x18,0x18,0x18,0x00,0x18,0x18,0x18,0x00}, // '|'
        {0x07,0x0C,0x0C,0x38,0x0C,0x0C,0x07,0x00}, // '}'
        {0x6E,0x3B,0x00,0x00,0x00,0x00,0x00,0x00}  // '~'
    };
    if (c < 32 || c > 126) return font[0];
    return font[c - 32];
}

class Window2D {
public:
    Window2D(const std::string& title, int width, int height,
             double x_min, double x_max, double y_min, double y_max)
        : width_(width), height_(height),
          x_min_(x_min), x_max_(x_max), y_min_(y_min), y_max_(y_max),
          pixels_(width * height, Palette::Background.to_u32()) {
        display_ = XOpenDisplay(nullptr);
        if (!display_) {
            open_ = false;
            return;
        }
        int screen = DefaultScreen(display_);
        Window root = RootWindow(display_, screen);
        window_ = XCreateSimpleWindow(display_, root, 50, 50, width_, height_, 1,
                                      BlackPixel(display_, screen), WhitePixel(display_, screen));
        XStoreName(display_, window_, title.c_str());
        XSelectInput(display_, window_, ExposureMask | KeyPressMask | ButtonPressMask |
                                        ButtonReleaseMask | PointerMotionMask | StructureNotifyMask);
        wm_delete_ = XInternAtom(display_, "WM_DELETE_WINDOW", False);
        XSetWMProtocols(display_, window_, &wm_delete_, 1);
        XMapWindow(display_, window_);
        gc_ = DefaultGC(display_, screen);

        Visual* visual = DefaultVisual(display_, screen);
        int depth = DefaultDepth(display_, screen);
        // XImage wraps our vector buffer directly
        ximage_ = XCreateImage(display_, visual, depth, ZPixmap, 0,
                               reinterpret_cast<char*>(pixels_.data()),
                               width_, height_, 32, width_ * sizeof(uint32_t));
        open_ = true;
    }

    ~Window2D() {
        if (display_) {
            if (ximage_) {
                ximage_->data = nullptr; // prevent XDestroyImage from calling free() on std::vector memory
                XDestroyImage(ximage_);
            }
            XDestroyWindow(display_, window_);
            XCloseDisplay(display_);
        }
    }

    bool is_open() const { return open_; }
    bool is_paused() const { return paused_; }
    bool consume_reset() { bool r = reset_requested_; reset_requested_ = false; return r; }
    bool consume_mode_toggle() { bool m = mode_toggle_; mode_toggle_ = false; return m; }
    bool mouse_down() const { return mouse_down_; }
    double mouse_world_x() const { return to_world_x(mouse_x_); }
    double mouse_world_y() const { return to_world_y(mouse_y_); }

    void poll_events() {
        if (!open_) return;
        while (XPending(display_) > 0) {
            XEvent ev;
            XNextEvent(display_, &ev);
            if (ev.type == ClientMessage) {
                if (static_cast<Atom>(ev.xclient.data.l[0]) == wm_delete_) {
                    open_ = false;
                }
            } else if (ev.type == KeyPress) {
                KeySym key = XLookupKeysym(&ev.xkey, 0);
                if (key == XK_Escape || key == XK_q || key == XK_Q) {
                    open_ = false;
                } else if (key == XK_space) {
                    paused_ = !paused_;
                } else if (key == XK_r || key == XK_R) {
                    reset_requested_ = true;
                } else if (key == XK_m || key == XK_M) {
                    mode_toggle_ = true;
                }
            } else if (ev.type == ButtonPress) {
                if (ev.xbutton.button == Button1) {
                    mouse_down_ = true;
                    mouse_x_ = ev.xbutton.x;
                    mouse_y_ = ev.xbutton.y;
                }
            } else if (ev.type == ButtonRelease) {
                if (ev.xbutton.button == Button1) {
                    mouse_down_ = false;
                }
            } else if (ev.type == MotionNotify) {
                mouse_x_ = ev.xmotion.x;
                mouse_y_ = ev.xmotion.y;
            }
        }
    }

    // Coordinate conversions
    int to_screen_x(double x) const {
        return static_cast<int>(std::round((x - x_min_) / (x_max_ - x_min_) * (width_ - 1)));
    }
    int to_screen_y(double y) const {
        return static_cast<int>(std::round((1.0 - (y - y_min_) / (y_max_ - y_min_)) * (height_ - 1)));
    }
    double to_world_x(int px) const {
        return x_min_ + (static_cast<double>(px) / (width_ - 1)) * (x_max_ - x_min_);
    }
    double to_world_y(int py) const {
        return y_min_ + (1.0 - static_cast<double>(py) / (height_ - 1)) * (y_max_ - y_min_);
    }
    int scale_len(double world_len) const {
        return static_cast<int>(std::round(world_len / (x_max_ - x_min_) * width_));
    }

    void clear(Color c = Palette::Background) {
        std::fill(pixels_.begin(), pixels_.end(), c.to_u32());
    }

    void put_pixel(int x, int y, Color c) {
        if (x >= 0 && x < width_ && y >= 0 && y < height_) {
            pixels_[y * width_ + x] = c.to_u32();
        }
    }

    void draw_grid(double step = 1.0) {
        for (double x = std::ceil(x_min_ / step) * step; x <= x_max_; x += step) {
            int sx = to_screen_x(x);
            for (int y = 0; y < height_; ++y) put_pixel(sx, y, Palette::Grid);
        }
        for (double y = std::ceil(y_min_ / step) * step; y <= y_max_; y += step) {
            int sy = to_screen_y(y);
            for (int x = 0; x < width_; ++x) put_pixel(x, sy, Palette::Grid);
        }
        int ox = to_screen_x(0.0);
        int oy = to_screen_y(0.0);
        for (int y = 0; y < height_; ++y) { put_pixel(ox, y, Palette::Axis); put_pixel(ox + 1, y, Palette::Axis); }
        for (int x = 0; x < width_; ++x) { put_pixel(x, oy, Palette::Axis); put_pixel(x, oy + 1, Palette::Axis); }
    }

    void draw_line_px(int x0, int y0, int x1, int y1, Color c, int thickness = 1) {
        int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy;
        int r = thickness / 2;

        while (true) {
            if (thickness <= 1) {
                put_pixel(x0, y0, c);
            } else {
                for (int oy = -r; oy <= r; ++oy)
                    for (int ox = -r; ox <= r; ++ox)
                        if (ox * ox + oy * oy <= r * r + 1)
                            put_pixel(x0 + ox, y0 + oy, c);
            }
            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    }

    void draw_line(double x0, double y0, double x1, double y1, Color c, int thickness = 2) {
        draw_line_px(to_screen_x(x0), to_screen_y(y0), to_screen_x(x1), to_screen_y(y1), c, thickness);
    }

    void fill_circle_px(int cx, int cy, int radius, Color c) {
        for (int y = -radius; y <= radius; ++y) {
            for (int x = -radius; x <= radius; ++x) {
                if (x * x + y * y <= radius * radius) {
                    put_pixel(cx + x, cy + y, c);
                }
            }
        }
    }

    void draw_circle_px(int cx, int cy, int radius, Color c, int thickness = 1) {
        int r_out = radius;
        int r_in = std::max(0, radius - thickness);
        for (int y = -r_out; y <= r_out; ++y) {
            for (int x = -r_out; x <= r_out; ++x) {
                int d2 = x * x + y * y;
                if (d2 <= r_out * r_out && d2 >= r_in * r_in) {
                    put_pixel(cx + x, cy + y, c);
                }
            }
        }
    }

    void fill_rect_px(int x, int y, int w, int h, Color fill_col, Color border_col) {
        for (int py = y; py < y + h; ++py) {
            for (int px = x; px < x + w; ++px) {
                if (px == x || px == x + w - 1 || py == y || py == y + h - 1)
                    put_pixel(px, py, border_col);
                else
                    put_pixel(px, py, fill_col);
            }
        }
    }

    void draw_text(int x, int y, const std::string& text, Color c = Palette::White, int scale = 1) {
        int cx = x;
        for (char ch : text) {
            const uint8_t* glyph = font8x8_basic(ch);
            for (int row = 0; row < 8; ++row) {
                uint8_t bits = glyph[row];
                for (int col = 0; col < 8; ++col) {
                    if (bits & (1 << col)) {
                        for (int sy = 0; sy < scale; ++sy)
                            for (int sx = 0; sx < scale; ++sx)
                                put_pixel(cx + col * scale + sx, y + row * scale + sy, c);
                    }
                }
            }
            cx += 8 * scale + scale;
        }
    }

    void present() {
        if (!open_) return;
        XPutImage(display_, window_, gc_, ximage_, 0, 0, 0, 0, width_, height_);
        XFlush(display_);
    }

private:
    int width_, height_;
    double x_min_, x_max_, y_min_, y_max_;
    std::vector<uint32_t> pixels_;

    Display* display_ = nullptr;
    Window window_ = 0;
    GC gc_ = 0;
    XImage* ximage_ = nullptr;
    Atom wm_delete_ = 0;

    bool open_ = false;
    bool paused_ = false;
    bool reset_requested_ = false;
    bool mode_toggle_ = false;
    bool mouse_down_ = false;
    int mouse_x_ = 0;
    int mouse_y_ = 0;
};

} // namespace gui

#endif // GUI_WINDOW_HPP

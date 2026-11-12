/**
 * @file odometry_sim.cpp
 * @brief Simulação Gráfica Interativa (X11) de Odometria para um Robô Móvel Diferencial.
 *
 * Compara em tempo real:
 * 1. Trajetória Real (Integração Exata por Centro Instantâneo de Curvatura - ICC)
 * 2. Odometria de Euler (1ª ordem, sem ruído)
 * 3. Odometria de Runge-Kutta de 2ª Ordem c/ Ruído nos Encoders (deriva acumulada / drift)
 */

#include "gui_window.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <random>
#include <thread>
#include <vector>

struct Pose2D {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0; // Orientação em radianos
};

struct DiffDriveParams {
    double wheel_radius = 0.05; // r [m] (5 cm)
    double wheel_base   = 0.30; // L [m] (30 cm entre as rodas)
};

double normalize_angle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// ============================================================================
// >>> O "CORE" DA ODOMETRIA DIFERENCIAL (ENCODERS DAS RODAS -> POSE 2D) <<<
// ============================================================================

/**
 * [CORE 1] Integração de Euler (1ª Ordem)
 * Assume que durante todo o passo dt o robô andou em linha reta na direção do
 * ângulo inicial (p.theta) e só girou no final. Gera pequeno erro numérico em curvas.
 */
Pose2D step_odometry_euler(const Pose2D& p, double ds_l, double ds_r, double L) {
    double ds = 0.5 * (ds_r + ds_l);     // Deslocamento linear do centro do robô
    double dtheta = (ds_r - ds_l) / L;   // Variação angular do chassi

    Pose2D next;
    next.x = p.x + ds * std::cos(p.theta);
    next.y = p.y + ds * std::sin(p.theta);
    next.theta = normalize_angle(p.theta + dtheta);
    return next;
}

/**
 * [CORE 2] Integração de Runge-Kutta de 2ª Ordem (Ponto Médio)
 * Usa a direção média do intervalo (theta + dtheta/2) para projetar (x, y).
 * Muito mais precisa que Euler com custo computacional quase idêntico!
 */
Pose2D step_odometry_rk2(const Pose2D& p, double ds_l, double ds_r, double L) {
    double ds = 0.5 * (ds_r + ds_l);
    double dtheta = (ds_r - ds_l) / L;

    Pose2D next;
    double mid_theta = p.theta + 0.5 * dtheta;
    next.x = p.x + ds * std::cos(mid_theta);
    next.y = p.y + ds * std::sin(mid_theta);
    next.theta = normalize_angle(p.theta + dtheta);
    return next;
}

/**
 * [CORE 3] Integração Exata por Arco Circular (ICC - Instantaneous Center of Curvature)
 * Modela o arco de circunferência de raio R = ds / dtheta que o robô descreve quando
 * as duas rodas mantêm velocidades constantes durante dt.
 */
Pose2D step_exact_icc(const Pose2D& p, double ds_l, double ds_r, double L) {
    double ds = 0.5 * (ds_r + ds_l);
    double dtheta = (ds_r - ds_l) / L;

    Pose2D next;
    if (std::abs(dtheta) < 1e-7) {
        next.x = p.x + ds * std::cos(p.theta);
        next.y = p.y + ds * std::sin(p.theta);
        next.theta = normalize_angle(p.theta + dtheta);
    } else {
        double R = ds / dtheta; // Raio instantâneo de curvatura
        next.x = p.x + R * (std::sin(p.theta + dtheta) - std::sin(p.theta));
        next.y = p.y - R * (std::cos(p.theta + dtheta) - std::cos(p.theta));
        next.theta = normalize_angle(p.theta + dtheta);
    }
    return next;
}

// ============================================================================
// VISUALIZAÇÃO GRÁFICA (X11 HEADER-ONLY)
// ============================================================================

struct HistoryRecord {
    double t;
    Pose2D truth;
    Pose2D odom_ideal;
    Pose2D odom_noisy;
};

void draw_robot_icon(gui::Window2D& win, const Pose2D& p, gui::Color col) {
    int cx = win.to_screen_x(p.x);
    int cy = win.to_screen_y(p.y);
    win.fill_circle_px(cx, cy, 9, col);
    win.draw_circle_px(cx, cy, 10, gui::Palette::White, 2);

    double hx = p.x + 0.25 * std::cos(p.theta);
    double hy = p.y + 0.25 * std::sin(p.theta);
    win.draw_line(p.x, p.y, hx, hy, gui::Palette::White, 2);
}

int main() {
    gui::Window2D win("Differential Drive Odometry Simulation (X11 Header-Only)", 960, 680, -3.8, 3.8, -2.6, 2.6);
    if (!win.is_open()) {
        std::cerr << "[Erro] Nao foi possivel abrir a janela X11 (verifique a variavel DISPLAY).\n";
        return 1;
    }

    DiffDriveParams robot{0.05, 0.30};
    const double dt = 0.03;
    const double period = 20.0;
    const double A = 2.6, B = 1.6;
    const double omega = 2.0 * M_PI / period;
    const double init_theta = std::atan2(2.0 * B * omega, A * omega);

    Pose2D truth, odom_euler, odom_noisy;
    std::vector<HistoryRecord> trail;
    double t = 0.0;

    // Ruído gaussiano (derrapagem / resolução de encoder) + 0.3% de erro sistemático no raio da roda direita
    std::mt19937 rng(42);
    std::normal_distribution<double> noise_dist(0.0, 0.0012);
    const double systematic_scale_err_r = 1.003;

    auto reset_sim = [&]() {
        t = 0.0;
        truth = {0.0, 0.0, init_theta};
        odom_euler = {0.0, 0.0, init_theta};
        odom_noisy = {0.0, 0.0, init_theta};
        trail.clear();
        rng.seed(42);
    };
    reset_sim();

    while (win.is_open()) {
        win.poll_events();
        if (win.consume_reset()) {
            reset_sim();
        }

        if (!win.is_paused()) {
            trail.push_back({t, truth, odom_euler, odom_noisy});
            if (trail.size() > 1400) {
                trail.erase(trail.begin());
            }

            // Curva em 8 (Lemniscata de Gerono) para gerar comandos (v, w)
            double vx = A * omega * std::cos(omega * t);
            double vy = 2.0 * B * omega * std::cos(2.0 * omega * t);
            double ax = -A * omega * omega * std::sin(omega * t);
            double ay = -4.0 * B * omega * omega * std::sin(2.0 * omega * t);

            double v_cmd = std::hypot(vx, vy);
            double w_cmd = (vx * ay - vy * ax) / (vx * vx + vy * vy);

            double v_l = v_cmd - (w_cmd * robot.wheel_base * 0.5);
            double v_r = v_cmd + (w_cmd * robot.wheel_base * 0.5);

            // Deslocamento real das rodas vs. leitura ruidosa dos encoders
            double ds_l_true = v_l * dt;
            double ds_r_true = v_r * dt;
            double ds_l_meas = ds_l_true + noise_dist(rng);
            double ds_r_meas = ds_r_true * systematic_scale_err_r + noise_dist(rng);

            // Executa os 3 métodos do CORE da Odometria
            truth      = step_exact_icc(truth, ds_l_true, ds_r_true, robot.wheel_base);
            odom_euler = step_odometry_euler(odom_euler, ds_l_true, ds_r_true, robot.wheel_base);
            odom_noisy = step_odometry_rk2(odom_noisy, ds_l_meas, ds_r_meas, robot.wheel_base);
            t += dt;
        }

        win.clear();
        win.draw_grid(1.0);

        for (size_t i = 1; i < trail.size(); ++i) {
            win.draw_line(trail[i - 1].odom_ideal.x, trail[i - 1].odom_ideal.y,
                          trail[i].odom_ideal.x, trail[i].odom_ideal.y, gui::Palette::Blue, 2);
            win.draw_line(trail[i - 1].odom_noisy.x, trail[i - 1].odom_noisy.y,
                          trail[i].odom_noisy.x, trail[i].odom_noisy.y, gui::Palette::Red, 2);
            win.draw_line(trail[i - 1].truth.x, trail[i - 1].truth.y,
                          trail[i].truth.x, trail[i].truth.y, gui::Palette::Green, 3);
        }

        draw_robot_icon(win, odom_noisy, gui::Palette::Red);
        draw_robot_icon(win, truth, gui::Palette::Green);

        // HUD Overlay
        win.fill_rect_px(16, 16, 465, 145, gui::Palette::PanelBg, gui::Palette::PanelBorder);
        win.draw_text(28, 28, "ODOMETRIA - ROBO MOVEL DIFERENCIAL", gui::Palette::Yellow, 1);

        char buf[128];
        std::snprintf(buf, sizeof(buf), "Real (ICC)       : x=%6.2fm  y=%6.2fm  th=%6.1f deg",
                      truth.x, truth.y, truth.theta * 180.0 / M_PI);
        win.draw_text(28, 52, buf, gui::Palette::Green, 1);

        std::snprintf(buf, sizeof(buf), "Odom Euler Ideal : x=%6.2fm  y=%6.2fm  err=%5.3fm",
                      odom_euler.x, odom_euler.y, std::hypot(truth.x - odom_euler.x, truth.y - odom_euler.y));
        win.draw_text(28, 72, buf, gui::Palette::Blue, 1);

        std::snprintf(buf, sizeof(buf), "Odom RK2 c/Ruido : x=%6.2fm  y=%6.2fm  err=%5.3fm",
                      odom_noisy.x, odom_noisy.y, std::hypot(truth.x - odom_noisy.x, truth.y - odom_noisy.y));
        win.draw_text(28, 92, buf, gui::Palette::Red, 1);

        std::snprintf(buf, sizeof(buf), "Tempo: %5.1fs | [SPACE] Pausar | [R] Reset | [Q/ESC] Sair", t);
        win.draw_text(28, 125, buf, gui::Palette::Gray, 1);

        win.present();
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    return 0;
}

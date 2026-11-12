/**
 * @file ik_sim.cpp
 * @brief Simulação Gráfica Interativa (X11) de Cinemática Inversa para um Robô Mini-Sumô no Dohyo.
 *
 * Em robôs móveis de combate (Mini-Sumô: 10x10cm, 500g, Dohyo de 77cm de diâmetro),
 * a Cinemática Inversa é dividida em dois níveis fundamentais ("O CORE DA COISA"):
 *
 *   [CORE 1] Lei de Controle Cinemático Inverso de Perseguição (Espaço do Dohyo -> Chassi):
 *            Dado o estado atual do nosso Mini-Sumô (x, y, theta) e a posição do oponente
 *            (target_x, target_y), calcula a velocidade linear desejada (v) e a velocidade
 *            angular de giro (omega) para alinhar a lâmina (wedge) e empurrar o adversário.
 *
 *   [CORE 2] Cinemática Inversa do Robô Diferencial (Espaço do Chassi -> Espaço das Rodas):
 *            Converte o par (v, omega) do centro de massa do robô nas velocidades lineares
 *            (v_L, v_R) e velocidades angulares (omega_L, omega_R em rad/s e RPM) exatas que
 *            os motores das rodas esquerda e direita devem executar.
 */

#include "gui_window.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <thread>
#include <utility>
#include <vector>

// Parâmetros físicos oficiais da categoria Mini-Sumô (500g, 10x10cm)
struct MiniSumoParams {
    double wheel_base   = 0.090; // L: Distância entre as rodas esquerda e direita (9.0 cm)
    double wheel_radius = 0.016; // r: Raio da roda de silicone (1.6 cm -> diâmetro 32mm)
    double body_size    = 0.100; // Largura/Comprimento oficial do chassi (10.0 cm)
    double max_wheel_v  = 1.200; // Velocidade máxima linear de cada roda (1.2 m/s)
    double dohyo_radius = 0.385; // Raio oficial do Dohyo de Mini-Sumô (Diâmetro = 77 cm -> R = 0.385 m)
};

struct Pose2D {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0; // Orientação da lâmina frontal [rad]
};

struct Twist2D {
    double v = 0.0;     // Velocidade linear frontal do chassi [m/s]
    double omega = 0.0; // Velocidade angular de giro do chassi [rad/s]
};

struct WheelCommand {
    double v_left = 0.0;      // Velocidade linear da roda esquerda [m/s]
    double v_right = 0.0;     // Velocidade linear da roda direita [m/s]
    double omega_left = 0.0;  // Velocidade angular do motor esquerdo [rad/s]
    double omega_right = 0.0; // Velocidade angular do motor direito [rad/s]
};

double normalize_angle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// ============================================================================
// >>> O "CORE" DA CINEMÁTICA INVERSA DO MINI-SUMÔ (PARTE 1 e PARTE 2) <<<
// ============================================================================

/**
 * [CORE 1] CONTROLADOR CINEMÁTICO INVERSO DE PERSEGUIÇÃO / ATAQUE
 * Converte o erro de posição no espaço cartesiano (Dohyo) em comandos de
 * velocidade do chassi (v, omega).
 *
 * - Se o oponente estiver muito desalinhado (|alpha| >= 90 graus), gira rápido no
 *   próprio eixo (v = 0) para não descrever curvas largas para fora do Dohyo.
 * - Quando a lâmina aponta para o oponente (alpha ~ 0), aplica velocidade máxima de ataque!
 */
Twist2D inverse_kinematics_pose_controller(const Pose2D& robot, double target_x, double target_y) {
    double dx = target_x - robot.x;
    double dy = target_y - robot.y;
    double rho = std::hypot(dx, dy); // Distância até o oponente [m]

    if (rho < 0.015) {
        return {0.0, 0.0};
    }

    // Ângulo absoluto do vetor que aponta do nosso robô para o oponente
    double target_heading = std::atan2(dy, dx);

    // Erro de alinhamento da lâmina frontal (alpha)
    double alpha = normalize_angle(target_heading - robot.theta);

    const double k_v = 2.8;     // Ganho de avanço proporcional à distância
    const double k_omega = 7.5; // Ganho de giro rápido para apontar a lâmina

    Twist2D cmd;
    double alignment_factor = std::max(0.0, std::cos(alpha));
    cmd.v = k_v * rho * alignment_factor;
    cmd.omega = k_omega * alpha;

    return cmd;
}

/**
 * [CORE 2] CINEMÁTICA INVERSA DO CHASSI DIFERENCIAL (CHASSI -> RODAS)
 * Equação fundamental que transforma a velocidade desejada do robô (v, omega)
 * nas velocidades individuais de cada roda/motor:
 *
 *   v_L = v - (omega * L) / 2       omega_L = v_L / r
 *   v_R = v + (omega * L) / 2       omega_R = v_R / r
 *
 * Também aplica saturação proporcional caso os motores atinjam o limite físico.
 */
WheelCommand inverse_kinematics_wheels(const Twist2D& chassis_cmd, const MiniSumoParams& params) {
    WheelCommand wheels;

    // --- Equação da Cinemática Inversa Diferencial ---
    wheels.v_left  = chassis_cmd.v - 0.5 * chassis_cmd.omega * params.wheel_base;
    wheels.v_right = chassis_cmd.v + 0.5 * chassis_cmd.omega * params.wheel_base;

    // Saturação proporcional para respeitar a velocidade máxima dos motores sem perder a curvatura
    double max_req = std::max(std::abs(wheels.v_left), std::abs(wheels.v_right));
    if (max_req > params.max_wheel_v) {
        double scale = params.max_wheel_v / max_req;
        wheels.v_left  *= scale;
        wheels.v_right *= scale;
    }

    // Conversão da velocidade linear da roda (m/s) para rotação do eixo do motor (rad/s)
    wheels.omega_left  = wheels.v_left  / params.wheel_radius;
    wheels.omega_right = wheels.v_right / params.wheel_radius;

    return wheels;
}

// ============================================================================
// CINEMÁTICA DIRETA (FÍSICA DA SIMULAÇÃO PARA ATUALIZAR A POSE NO DOHYO)
// ============================================================================
Pose2D forward_kinematics_step(const Pose2D& p, const WheelCommand& w, double L, double dt) {
    double v = 0.5 * (w.v_right + w.v_left);
    double omega = (w.v_right - w.v_left) / L;

    Pose2D next;
    double mid_theta = p.theta + 0.5 * omega * dt;
    next.x = p.x + v * std::cos(mid_theta) * dt;
    next.y = p.y + v * std::sin(mid_theta) * dt;
    next.theta = normalize_angle(p.theta + omega * dt);
    return next;
}

// ============================================================================
// VISUALIZAÇÃO GRÁFICA (X11 HEADER-ONLY)
// ============================================================================
void draw_minisumo_chassis(gui::Window2D& win, const Pose2D& p, double size, gui::Color body_col, gui::Color blade_col) {
    double h = size * 0.5;
    double c = std::cos(p.theta), s = std::sin(p.theta);

    auto rot = [&](double lx, double ly) -> std::pair<double, double> {
        return {p.x + lx * c - ly * s, p.y + lx * s + ly * c};
    };

    auto p_fl = rot( h,  h); // Frontal esquerdo
    auto p_fr = rot( h, -h); // Frontal direito
    auto p_br = rot(-h, -h); // Traseiro direito
    auto p_bl = rot(-h,  h); // Traseiro esquerdo

    win.draw_line(p_fl.first, p_fl.second, p_fr.first, p_fr.second, blade_col, 5); // Lâmina frontal (Wedge)
    win.draw_line(p_fr.first, p_fr.second, p_br.first, p_br.second, body_col, 3);
    win.draw_line(p_br.first, p_br.second, p_bl.first, p_bl.second, body_col, 3);
    win.draw_line(p_bl.first, p_bl.second, p_fl.first, p_fl.second, body_col, 3);

    auto p_tip = rot(h * 1.35, 0.0);
    win.draw_line(p.x, p.y, p_tip.first, p_tip.second, blade_col, 2);
}

int main() {
    gui::Window2D win("Mini-Sumo Inverse Kinematics Arena (X11 Header-Only)", 960, 720, -0.55, 0.55, -0.42, 0.42);
    if (!win.is_open()) {
        std::cerr << "[Erro] Nao foi possivel abrir a janela X11 (verifique a variavel DISPLAY).\n";
        return 1;
    }

    MiniSumoParams params;
    Pose2D sumo{-0.22, -0.15, 0.0};
    Pose2D opponent{0.18, 0.12, M_PI};
    bool manual_opponent = false;
    double t = 0.0;
    const double dt = 0.016;

    std::vector<Pose2D> sumo_trail;
    std::vector<Pose2D> opp_trail;

    while (win.is_open()) {
        win.poll_events();
        if (win.consume_mode_toggle()) {
            manual_opponent = !manual_opponent;
        }
        if (win.consume_reset()) {
            t = 0.0;
            sumo = {-0.22, -0.15, 0.0};
            opponent = {0.18, 0.12, M_PI};
            sumo_trail.clear();
            opp_trail.clear();
        }

        if (win.mouse_down()) {
            opponent.x = win.mouse_world_x();
            opponent.y = win.mouse_world_y();
        } else if (!win.is_paused() && !manual_opponent) {
            opponent.x = 0.24 * std::cos(1.1 * t);
            opponent.y = 0.22 * std::sin(1.65 * t);
            opponent.theta = std::atan2(sumo.y - opponent.y, sumo.x - opponent.x);
        }

        // Executa os 2 níveis do CORE da Cinemática Inversa
        Twist2D twist_cmd = inverse_kinematics_pose_controller(sumo, opponent.x, opponent.y);
        WheelCommand wheel_cmd = inverse_kinematics_wheels(twist_cmd, params);

        if (!win.is_paused()) {
            sumo = forward_kinematics_step(sumo, wheel_cmd, params.wheel_base, dt);
            sumo_trail.push_back(sumo);
            opp_trail.push_back(opponent);
            if (sumo_trail.size() > 250) sumo_trail.erase(sumo_trail.begin());
            if (opp_trail.size() > 250) opp_trail.erase(opp_trail.begin());
            t += dt;
        }

        win.clear();
        win.draw_grid(0.10); // Grade a cada 10cm (tamanho de 1 Mini-Sumô)

        // Desenha o Dohyo oficial (Raio = 38.5cm) e a Tawara (borda branca)
        int cx = win.to_screen_x(0.0);
        int cy = win.to_screen_y(0.0);
        int r_px = win.scale_len(params.dohyo_radius);
        win.draw_circle_px(cx, cy, r_px + 6, gui::Palette::White, 7);

        // Shikiri-sen (linhas centrais de partida)
        win.draw_line(-0.08, -0.04, -0.08, 0.04, gui::Palette::Orange, 3);
        win.draw_line( 0.08, -0.04,  0.08, 0.04, gui::Palette::Orange, 3);

        for (size_t i = 1; i < opp_trail.size(); ++i) {
            win.draw_line(opp_trail[i - 1].x, opp_trail[i - 1].y, opp_trail[i].x, opp_trail[i].y, gui::Palette::Red, 1);
        }
        for (size_t i = 1; i < sumo_trail.size(); ++i) {
            win.draw_line(sumo_trail[i - 1].x, sumo_trail[i - 1].y, sumo_trail[i].x, sumo_trail[i].y, gui::Palette::Green, 2);
        }

        win.draw_line(sumo.x, sumo.y, opponent.x, opponent.y, gui::Palette::Axis, 1);

        draw_minisumo_chassis(win, opponent, params.body_size, gui::Palette::Red, gui::Palette::Orange);
        draw_minisumo_chassis(win, sumo, params.body_size, gui::Palette::Green, gui::Palette::Yellow);

        // HUD Informativo
        win.fill_rect_px(14, 14, 535, 175, gui::Palette::PanelBg, gui::Palette::PanelBorder);
        win.draw_text(26, 26, "CINEMATICA INVERSA - MINI-SUMO NO DOHYO (77cm)", gui::Palette::Yellow, 1);

        char buf[128];
        std::snprintf(buf, sizeof(buf), "Pose Mini-Sumo : x=%+5.2fm  y=%+5.2fm  theta=%+6.1f deg",
                      sumo.x, sumo.y, sumo.theta * 180.0 / M_PI);
        win.draw_text(26, 48, buf, gui::Palette::Green, 1);

        std::snprintf(buf, sizeof(buf), "Alvo/Oponente  : x=%+5.2fm  y=%+5.2fm  dist = %5.2fm",
                      opponent.x, opponent.y, std::hypot(opponent.x - sumo.x, opponent.y - sumo.y));
        win.draw_text(26, 68, buf, gui::Palette::Red, 1);

        std::snprintf(buf, sizeof(buf), "[CORE 1] Chassi: v = %+5.2f m/s   | omega = %+6.2f rad/s",
                      twist_cmd.v, twist_cmd.omega);
        win.draw_text(26, 92, buf, gui::Palette::Cyan, 1);

        double rpm_l = wheel_cmd.omega_left * 60.0 / (2.0 * M_PI);
        double rpm_r = wheel_cmd.omega_right * 60.0 / (2.0 * M_PI);
        std::snprintf(buf, sizeof(buf), "[CORE 2] Rodas : vL=%+5.2fm/s (%+4.0fRPM) | vR=%+5.2fm/s (%+4.0fRPM)",
                      wheel_cmd.v_left, rpm_l, wheel_cmd.v_right, rpm_r);
        win.draw_text(26, 112, buf, gui::Palette::Orange, 1);

        win.draw_text(26, 138, "Arraste o MOUSE com botao esquerdo para posicionar o Oponente!", gui::Palette::White, 1);
        win.draw_text(26, 160, "[M] Modo Auto/Manual | [SPACE] Pausar | [R] Reset | [Q] Sair", gui::Palette::Gray, 1);

        win.present();
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    return 0;
}

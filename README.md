# Simulações Gráficas em C++17: Odometria & Cinemática Inversa (Mini-Sumô)

Este projeto implementa duas simulações gráficas interativas em **C++17** usando apenas a biblioteca padrão e um motor gráfico 2D **100% header-only** (`include/gui_window.hpp`) construído sobre a `libX11` nativa do Linux (funciona diretamente em qualquer distribuição Linux com X11 ou Wayland/XWayland, sem necessidade de instalar bibliotecas externas).

1. **Simulação de Odometria (`odometry_sim`)**: Robô móvel diferencial (2 rodas) percorrendo uma trajetória em formato de oito (lemniscata de Gerono). Compara em tempo real a pose real (integração exata por Centro Instantâneo de Curvatura - ICC) contra a odometria de Euler de 1ª ordem e a odometria de Runge-Kutta de 2ª ordem com ruído nos encoders.
2. **Simulação de Cinemática Inversa para Robô Mini-Sumô (`ik_sim`)**: Simula um robô oficial da categoria **Mini-Sumô** ($10\times 10\text{ cm}$, $500\text{ g}$, distância entre rodas $L = 9\text{ cm}$, rodas de raio $r = 1.6\text{ cm}$) dentro de um **Dohyo oficial** ($77\text{ cm}$ de diâmetro) perseguindo e alinhando sua lâmina frontal (*wedge*) contra um oponente móvel.

---

## Estrutura do Projeto

```text
robotics-sim-cpp/
├── include/
│   └── gui_window.hpp   # Motor gráfico 2D header-only (X11 puro + fonte 8x8 embutida)
├── src/
│   ├── odometry_sim.cpp # Simulador gráfico de Odometria (Robô Diferencial)
│   └── ik_sim.cpp       # Simulador gráfico de Cinemática Inversa (Mini-Sumô no Dohyo)
├── Makefile             # Automação de compilação e execução
└── README.md            # Documentação e explicação matemática
```

---

## Como Compilar e Executar

### 1. Compilar ambos os simuladores

```bash
make
```

### 2. Executar a Simulação do Mini-Sumô (Cinemática Inversa)

```bash
make run-ik
# ou diretamente: ./bin/ik_sim
```

### 3. Executar a Simulação de Odometria

```bash
make run-odom
# ou diretamente: ./bin/odometry_sim
```

### Controles Interativos nas Janelas Gráficas

- **`Clique Esquerdo + Arrastar`** (em `ik_sim`): Move o robô oponente manualmente pelo Dohyo em tempo real.
- **`M`** (em `ik_sim`): Alterna o oponente entre movimento evasivo automático e modo manual parado.
- **`Espaço`**: Pausa / despausa a simulação.
- **`R`**: Reinicia a simulação e limpa as trilhas.
- **`Q` ou `ESC`**: Fecha a janela gráfica.

# ==============================================================================
# Makefile - Simulações Gráficas (X11 Header-Only) de Odometria e Mini-Sumô IK
# Dependências: Biblioteca padrão do C++17 + libX11 nativa do Linux
# ==============================================================================

CXX      ?= g++
CXXFLAGS := -std=c++17 -Wall -Wextra -Wpedantic -O2 -Iinclude
LDFLAGS  := -lX11 -lm

BIN_DIR  := bin
ODOM_BIN := $(BIN_DIR)/odometry_sim
IK_BIN   := $(BIN_DIR)/ik_sim
HEADERS  := include/gui_window.hpp

.PHONY: all run-odom run-ik clean help

all: $(ODOM_BIN) $(IK_BIN)

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

$(ODOM_BIN): src/odometry_sim.cpp $(HEADERS) | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDFLAGS)

$(IK_BIN): src/ik_sim.cpp $(HEADERS) | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDFLAGS)

run-odom: $(ODOM_BIN)
	./$(ODOM_BIN)

run-ik: $(IK_BIN)
	./$(IK_BIN)

clean:
	rm -rf $(BIN_DIR)

help:
	@echo "Alvos disponíveis no Makefile:"
	@echo "  make          - Compila ambos os simuladores gráficos (odometry_sim e ik_sim)"
	@echo "  make run-odom - Abre a janela gráfica X11 da simulação de Odometria"
	@echo "  make run-ik   - Abre a janela gráfica X11 interativa do Mini-Sumô no Dohyo"
	@echo "  make clean    - Remove os binários compilados"

#!/usr/bin/env python3
"""
Leitura do Bar30 (MS5837-30BA) ligado na porta I2C da Navigator.

- Ao iniciar, mede a pressão da superfície e usa como profundidade zero.
- Depois mostra pressão, temperatura e profundidade no terminal.
- Rodar SEM sudo:  python3 bar30_leitura.py     (Ctrl+C para sair)
"""

import time

import ms5837

# ---------------- Configuração ----------------
BARRAMENTO = 6            # porta I2C externa da Navigator = /dev/i2c-6
FREQ_HZ = 5               # leituras por segundo (máximo ~20 com OSR_8192)
OSR = ms5837.OSR_8192     # oversampling máximo: menos ruído, ~41 ms por leitura
DENSIDADE = 997.0         # kg/m³: 997 água doce, 1029 água salgada
G = 9.80665               # gravidade padrão (m/s²)
AMOSTRAS_ZERO = 10        # leituras usadas para medir a pressão da superfície


def iniciar_sensor():
    """Abre o I2C e confere se o Bar30 responde. Devolve o sensor ou None."""
    sensor = ms5837.MS5837_30BA(BARRAMENTO)
    try:
        ok = sensor.init()
    except OSError as erro:                  # ninguém respondeu no endereço 0x76
        print(f"Erro de I2C: {erro}")
        ok = False
    if not ok:
        print(f"Bar30 não encontrado no I2C {BARRAMENTO} (endereço 0x76).")
        print(f"Confira com: i2cdetect -y {BARRAMENTO}")
        return None
    return sensor


def medir_superficie(sensor):
    """Média de algumas leituras: é a pressão que vira profundidade zero (mbar)."""
    soma = 0.0
    for _ in range(AMOSTRAS_ZERO):
        sensor.read(OSR)
        soma += sensor.pressure()
    return soma / AMOSTRAS_ZERO


def ler(sensor, p_superficie):
    """Uma leitura. Devolve (pressão em mbar, temperatura em °C, profundidade em m)."""
    sensor.read(OSR)
    pressao = sensor.pressure()              # mbar
    temperatura = sensor.temperature()       # °C
    profundidade = (pressao - p_superficie) * 100 / (DENSIDADE * G)   # mbar -> Pa, h = ΔP / (ρ·g)
    return pressao, temperatura, profundidade


def main():
    sensor = iniciar_sensor()
    if sensor is None:
        return

    p_superficie = medir_superficie(sensor)
    print(f"Pressão da superfície (profundidade 0): {p_superficie:.1f} mbar")
    print("Lendo... Ctrl+C para sair.")

    periodo = 1.0 / FREQ_HZ
    try:
        while True:
            inicio = time.monotonic()
            try:
                pressao, temperatura, profundidade = ler(sensor, p_superficie)
                print(f"Pressão: {pressao:8.1f} mbar | "
                      f"Temperatura: {temperatura:5.2f} °C | "
                      f"Profundidade: {profundidade:7.3f} m")
            except OSError as erro:          # falha isolada de I2C: avisa e continua
                print(f"Falha na leitura: {erro}")
            time.sleep(max(0.0, periodo - (time.monotonic() - inicio)))
    except KeyboardInterrupt:
        print("\nEncerrado.")


if __name__ == "__main__":
    main()

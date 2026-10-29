#!/usr/bin/env python3
"""
Controle simples de um servo Hitec HS-5496MH pela Navigator (Blue Robotics).

- Ao iniciar, coloca o servo no zero (pulso de 1500 us).
- Depois lê comandos do teclado e envia ao servo.
- Rodar SEM sudo:  python3 servo_zero.py
"""

import time

import bluerobotics_navigator as navigator
from bluerobotics_navigator import PwmChannel

# ---------------- Configuração ----------------
CANAL = PwmChannel.Ch1   # saída PWM da Navigator onde o servo está ligado (1 a 16)
FREQ_HZ = 50             # frequência padrão de servo: um pulso a cada 20 ms
ZERO_US = 1500           # largura de pulso do "zero" (neutro/centro do servo)
MIN_US = 1000            # limite inferior seguro (alargue aos poucos depois de testar)
MAX_US = 2000            # limite superior seguro

PERIODO_US = 1_000_000 / FREQ_HZ   # 20 000 us a 50 Hz
PASSOS = 4096                       # o PCA9685 divide cada período em 4096 passos


def us_para_contagem(pulso_us):
    """Converte a largura de pulso (us) no valor de 12 bits que a placa espera."""
    return int(round(pulso_us / PERIODO_US * PASSOS))


def mover(pulso_us):
    """Limita o pulso à faixa segura, envia ao servo e devolve o pulso enviado."""
    pulso_us = int(round(max(MIN_US, min(MAX_US, pulso_us))))
    navigator.set_pwm_channel_value(CANAL, us_para_contagem(pulso_us))
    return pulso_us


def main():
    navigator.init()
    navigator.set_pwm_freq_hz(FREQ_HZ)
    atual = mover(ZERO_US)            # grava o zero ANTES de ligar as saídas
    navigator.set_pwm_enable(True)
    print(f"Servo no zero: {atual} us (contagem {us_para_contagem(atual)})")

    print("Comandos: <us> ex. 1600 | +N / -N passo relativo | z zero | q sair")
    try:
        while True:
            cmd = input(f"[{atual} us] > ").strip().lower()
            if not cmd:
                continue
            if cmd == "q":
                break
            if cmd == "z":
                alvo = ZERO_US
            else:
                try:
                    valor = float(cmd)
                except ValueError:
                    print("Comando inválido.")
                    continue
                alvo = atual + valor if cmd[0] in "+-" else valor

            atual = mover(alvo)
            if atual != round(alvo):
                print(f"Fora da faixa [{MIN_US}, {MAX_US}] us: limitado a {atual} us")
            print(f"Enviado: {atual} us (contagem {us_para_contagem(atual)})")
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        mover(ZERO_US)                # volta ao zero ao sair
        time.sleep(0.5)               # tempo para o servo chegar lá
        navigator.set_pwm_enable(False)
        print("Servo de volta ao zero, saídas PWM desligadas.")


if __name__ == "__main__":
    main()

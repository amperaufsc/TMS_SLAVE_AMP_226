import numpy as np
import matplotlib.pyplot as plt
from numpy.polynomial.polynomial import Polynomial

# ============================================================
# Dados do datasheet: MF52 NTC 10KΩ (B3950, 25/50°C)
# Fonte: pj2-mf52type-1554.pdf - Tabela R-T
# ============================================================

# Temperatura (°C) e Resistência (KΩ) do datasheet
temperatura = np.array([
    -30, -25, -20, -15, -10, -5,
    0, 5, 10, 15, 20, 25, 30, 35, 40, 45,
    50, 55, 60, 65, 70, 75, 80, 85, 90, 95,
    100, 105, 110
])

# Resistência em Ohm (convertido de KΩ do datasheet)
resistencia_ntc = np.array([
    181.70, 133.30, 98.88, 74.10, 56.06, 42.80,
    32.96, 25.58, 20.00, 15.76, 12.51, 10.00, 8.048, 6.518, 5.312, 4.354,
    3.588, 2.974, 2.476, 2.072, 1.743, 1.473, 1.250, 1.065, 0.911, 0.7824,
    0.6744, 0.5836, 0.5066
]) * 1000  # Converter KΩ para Ω
# NOTA: O valor de 0°C (32.96 KΩ) foi corrigido - o PDF tinha um erro de extração
# nessa célula (mostrava 98.96 que era da coluna de 100KΩ B4000).
# Valor validado pela equação Beta: R = R25 * exp(B * (1/T - 1/T0)) ≈ 33.62 KΩ

# Resistor fixo do divisor de tensão (10kΩ)
R_fixo = 10000  # Ohm

# Tensão de referência do ADC (3.3V)
V_ref = 3.3

# Calculando a tensão de saída do divisor de tensão
# Topologia: V_ref --- R_fixo --- V_adc --- NTC --- GND
V_adc = V_ref * resistencia_ntc / (resistencia_ntc + R_fixo)

# Ajuste polinomial de 4ª ordem (Temperatura = f(V_adc))
coeficientes = np.polyfit(V_adc, temperatura, 4)

# Criando o polinômio a partir dos coeficientes
polinomio = Polynomial(coeficientes[::-1])  # np.polyfit retorna coeficientes no formato reverso

# Gerando valores para plotagem
V_adc_plot = np.linspace(min(V_adc), max(V_adc), 500)
temperatura_ajustada = polinomio(V_adc_plot)

# ============================================================
# Calculando o erro do ajuste
# ============================================================
temperatura_estimada = polinomio(V_adc)
erro = temperatura - temperatura_estimada
erro_max = np.max(np.abs(erro))
erro_rms = np.sqrt(np.mean(erro**2))

# ============================================================
# Plotagem
# ============================================================
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

# Gráfico principal: dados vs ajuste
ax1.scatter(V_adc, temperatura, label="Dados do Datasheet (MF52 10K B3950)", color="red", zorder=5)
ax1.plot(V_adc_plot, temperatura_ajustada, label=f"Ajuste Polinomial (4ª ordem)", color="blue")
ax1.set_xlabel("Tensão no ADC (V)")
ax1.set_ylabel("Temperatura (°C)")
ax1.set_title("MF52 NTC 10K (B3950) - Ajuste Polinomial: Temperatura vs. Tensão no ADC")
ax1.legend()
ax1.grid(True, alpha=0.3)

# Gráfico de erro
ax2.stem(V_adc, erro, linefmt='r-', markerfmt='ro', basefmt='k-')
ax2.set_xlabel("Tensão no ADC (V)")
ax2.set_ylabel("Erro (°C)")
ax2.set_title(f"Erro do Ajuste | Max: {erro_max:.2f}°C | RMS: {erro_rms:.2f}°C")
ax2.grid(True, alpha=0.3)
ax2.axhline(y=0, color='k', linewidth=0.5)

plt.tight_layout()
plt.show()

# ============================================================
# Exibindo resultados
# ============================================================
print("=" * 60)
print("MF52 NTC 10K (B3950) - Coeficientes do Polinômio de 4ª ordem")
print("=" * 60)
print(f"\nT(V) = a·V⁴ + b·V³ + c·V² + d·V + e\n")
print(f"  a = {coeficientes[0]:.6f}")
print(f"  b = {coeficientes[1]:.6f}")
print(f"  c = {coeficientes[2]:.6f}")
print(f"  d = {coeficientes[3]:.6f}")
print(f"  e = {coeficientes[4]:.6f}")
print(f"\nErro máximo: {erro_max:.2f}°C")
print(f"Erro RMS:    {erro_rms:.2f}°C")
print(f"\nFaixa de tensão ADC: {min(V_adc):.4f}V a {max(V_adc):.4f}V")
print(f"Faixa de temperatura: {min(temperatura)}°C a {max(temperatura)}°C")

# ============================================================
# Código C para firmware STM32
# ============================================================
print("\n" + "=" * 60)
print("Código C para o firmware STM32:")
print("=" * 60)
print(f"""
// MF52 NTC 10K (B3950) - Polinômio de 4ª ordem
// T(V) = a*V^4 + b*V^3 + c*V^2 + d*V + e
// Faixa válida: {min(V_adc):.4f}V a {max(V_adc):.4f}V ({min(temperatura)}°C a {max(temperatura)}°C)
float ntc_voltage_to_temperature(float v_adc) {{
    const float a = {coeficientes[0]:.6f}f;
    const float b = {coeficientes[1]:.6f}f;
    const float c = {coeficientes[2]:.6f}f;
    const float d = {coeficientes[3]:.6f}f;
    const float e = {coeficientes[4]:.6f}f;

    float v2 = v_adc * v_adc;
    float v3 = v2 * v_adc;
    float v4 = v3 * v_adc;

    return a*v4 + b*v3 + c*v2 + d*v_adc + e;
}}
""")
